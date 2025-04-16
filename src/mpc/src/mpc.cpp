#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <cmath>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <Iir.h>
#include <acado/acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/acado_gnuplot.hpp>
#include <acado/acado_code_generation.hpp>
using namespace std;

class LowPassFilter
{
private:
    double alpha;
    double last_output;

public:
    // Constructor
    LowPassFilter(double alpha_value) : alpha(alpha_value), last_output(0.0) {}

    // Filter a new value
    double filter(double input)
    {
        double output = alpha * input + (1 - alpha) * last_output;
        last_output = output;
        return output;
    }
};

class ButterworthLowPassFilter
{
private:
    double fs; // 采样频率
    double fc; // 截止频率
    double a0, a1, a2, b1, b2;
    double prevX1, prevX2, prevY1, prevY2;

public:
    ButterworthLowPassFilter(double sampleRate, double cutoffFrequency)
        : fs(sampleRate), fc(cutoffFrequency), prevX1(0), prevX2(0), prevY1(0), prevY2(0)
    {
        calculateCoefficients();
    }

    void calculateCoefficients()
    {
        double ita = 1.0 / tan(M_PI * fc / fs);
        double q = sqrt(2.0);
        double a0_temp = 1.0 / (1.0 + q * ita + ita * ita);
        a0 = a0_temp;
        a1 = 2 * a0_temp;
        a2 = a0_temp;
        b1 = 2.0 * a0_temp * (1 - ita * ita);
        b2 = a0_temp * (1 - q * ita + ita * ita);
    }

    double filter(double input)
    {
        double output = a0 * input + a1 * prevX1 + a2 * prevX2 - b1 * prevY1 - b2 * prevY2;
        prevX2 = prevX1;
        prevX1 = input;
        prevY2 = prevY1;
        prevY1 = output;
        return output;
    }

    void reset()
    {
        prevX1 = prevX2 = prevY1 = prevY2 = 0;
    }
};

class MyController
{
private:
    ros::NodeHandle nh;
    Eigen::Vector3d u;
    double x_real, x_real_last;
    double y_real, y_real_last;
    double z_real, z_real_last;
    double x_real_derivative, x_filtered_deri;
    double y_real_derivative, y_filtered_deri;
    double z_real_derivative, z_filtered_deri;
    double time_now, time_last, time_pass;
    bool first_time_in_fun, loss_target;

    // ButterworthLowPassFilter filter_for_deri; // 二阶巴特沃斯LPF
    LowPassFilter filter_for_imgx, filter_for_imgy, filter_for_imgz;
    //MPCController mpc_controller;
    ros::Subscriber px4_state_sub, sub;
    friend class TripleAxisController;
    double loss_or_not_;
    int px4_state;
    Iir::Butterworth::LowPass<2> filter_4_for_derix;
    Iir::Butterworth::LowPass<2> filter_4_for_deriy;
    Iir::Butterworth::LowPass<2> filter_4_for_deriz;

public:
    // 构造函数
    MyController()
        : nh(),
          px4_state(1),
          x_real(0.0),
          x_real_last(0.0),
          x_real_derivative(0.0),
          y_real(0.0),
          y_real_last(0.0),
          y_real_derivative(0.0),
          z_real(0.0),
          z_real_last(0.0),
          z_real_derivative(0.0),
          time_now(0.0),
          time_last(0.0),
          time_pass(0.0),
          filter_for_imgx(0.97),
          filter_for_imgy(0.97),
          filter_for_imgz(0.97),
          x_filtered_deri(0),
          y_filtered_deri(0),
          z_filtered_deri(0),
          loss_target(true),
          loss_or_not_(1),
          first_time_in_fun(true)
    {
        u.setZero();
        px4_state_sub = nh.subscribe("/px4_state_pub", 1, &MyController::StateCallback, this);
        sub = nh.subscribe("/point_with_fixed_delay", 1, &MyController::callback, this);
        filter_4_for_derix.setup(20, 2.7);
        filter_4_for_deriy.setup(20, 2.7);
        filter_4_for_deriz.setup(20, 2.7);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        Eigen::Vector3d xyz_1;
        xyz_1 << msg->data[0], msg->data[1], msg->data[2];
        cal_single_axis_ctrl_input(xyz_1, msg->data[4]);
    }

    void cal_single_axis_ctrl_input(Eigen::Vector3d xyz_1, double loss_or_not)
    {
        loss_or_not_ = loss_or_not;
        x_real = xyz_1(0);
        x_real = filter_for_imgx.filter(x_real);
        y_real = xyz_1(1);
        y_real = filter_for_imgy.filter(y_real);
        z_real = xyz_1(2);
        z_real = filter_for_imgz.filter(z_real);
        time_now = ros::Time::now().toSec();
        time_pass = time_now - time_last;
        x_real_derivative = (x_real - x_real_last) / time_pass; // 0.05 seconds = 50ms
        y_real_derivative = (y_real - y_real_last) / time_pass; // 0.05 seconds = 50ms
        z_real_derivative = (z_real - z_real_last) / time_pass; // 0.05 seconds = 50ms
        x_filtered_deri = filter_4_for_derix.filter(x_real_derivative);
        y_filtered_deri = filter_4_for_deriy.filter(y_real_derivative);
        z_filtered_deri = filter_4_for_deriz.filter(z_real_derivative);
        function(loss_or_not_);
        x_real_last = x_real;
        y_real_last = y_real;
        z_real_last = z_real;
        time_last = time_now;
    }

    void function(double loss_or_not)
    {
        if (loss_or_not == 1 && loss_target == false) // 从能看到目标到看不到目标
        {
            loss_target = true;
        }
        if (loss_or_not == 0 && loss_target == true) // 从不能看到目标到能看到目标
        {
            loss_target = false;
            first_time_in_fun = true;
        }
        if (first_time_in_fun)
        {
            time_pass = 0.05;
            x_filtered_deri = 0;
            y_filtered_deri = 0;
            z_filtered_deri = 0;
            Eigen::Vector3d xyz_1;
            xyz_1 << x_real, y_real, z_real;
            Eigen::Vector3d xyz_2;
            xyz_2 << x_filtered_deri, y_filtered_deri, z_filtered_deri;
            first_time_in_fun = false;
        }
        else
        {
            Eigen::Vector3d xyz_1;
            xyz_1 << x_real, y_real, z_real;
            Eigen::Vector3d xyz_2;
            xyz_2 << x_filtered_deri, y_filtered_deri, z_filtered_deri;
        }
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        px4_state = msg->data;
        if (msg->data != 3) // 不在cmd模式下时，控制量为0；
        {
            u.setZero();
        }
    }

    void spin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_controller_node");
    MyController controller;
    // TODO: Add ROS subscribers, publishers, and service servers/clients as needed
    controller.spin();
    return 0;
}
