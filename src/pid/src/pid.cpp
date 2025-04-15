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
using namespace std;

class PID2Controller
{
private:
    const double k_i = -0.2;
    const double k_p = -4;
    const double k_d = -4;
    const double T_c = 0.05;
    double x_bias = -0;
    double y_bias = -0.1;
    double z_bias = -0.2;
    double y1_real_bias = 0;
    ros::NodeHandle nh;

public:
    PID2Controller() // 构造函数
    {
    }


    double adjustBias(double value, int which_axis, bool use_bias) // which_axis: 0 for x, 1 for y, 2 for z
    {
        if (use_bias)
        {
            if (which_axis == 0)
            {
                return value + x_bias;
            }
            else if (which_axis == 1)
            {
                return value + y_bias;
            }
            else
            {
                return value + z_bias;
            }
        }
        else
        {
            return value;
        }
    }

    double limitControl(double controlValue, int which_axis)
    {
        double limit = (which_axis != 2) ? 3 : 3; // Limit is 3 for x and y, 4 for z axis.
        if (abs(controlValue) >= limit)
        {
            controlValue = limit * controlValue / abs(controlValue); // Apply limit
        }
        return controlValue;
    }

    double limitIntegral(double IntegralValue, int which_axis)
    {
        double limit = (which_axis != 2) ? 1 : 1; // Limit is 1.5 for x and y, 2 for z axis.
        if (abs(IntegralValue) >= limit)
        {
            IntegralValue = limit * IntegralValue / abs(IntegralValue); // Apply limit
        }
        return IntegralValue;
    }

    void computeControl(double y1_real_slow, double y2_derivative_sampling, double &u, double &integral_error, bool use_bias, int which_axis)
    {
        y1_real_bias = adjustBias(y1_real_slow, which_axis, use_bias);
        integral_error = integral_error + y1_real_bias * T_c;
        integral_error = limitIntegral(integral_error, which_axis);
        u = k_i * integral_error + k_p * y1_real_bias + k_d * y2_derivative_sampling;
        u = limitControl(u, which_axis);
    }
};

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
    double u, integral_error;
    double y_real, y_real_last;
    double y_real_derivative, y_filtered_deri;
    double time_now, time_last, time_pass;
    bool first_time_in_fun, loss_target, use_bias_;
    ros::NodeHandle nh;

    // ButterworthLowPassFilter filter_for_deri; // 二阶巴特沃斯LPF
    LowPassFilter filter_for_img;
    PID2Controller pid2controller;
    ros::Subscriber px4_state_sub;
    friend class TripleAxisController;
    double loss_or_not_;
    int which_axis_;
    int px4_state;
    Iir::Butterworth::LowPass<2> filter_4_for_deri;

public:
    // 构造函数
    MyController()
        : nh("~"),
          px4_state(1),
          y_real(0.0),
          u(0.0),
          y_real_last(0.0),
          y_real_derivative(0.0),
          time_now(0.0),
          time_last(0.0),
          time_pass(0.0),
          filter_for_img(0.9),
          // filter_for_deri(20, 3),
          y_filtered_deri(0),
          integral_error(0),
          loss_target(true),
          loss_or_not_(1),
          use_bias_(1),
          which_axis_(0),
          first_time_in_fun(true)
    {
        px4_state_sub = nh.subscribe("/px4_state_pub", 1, &MyController::StateCallback, this);
        filter_4_for_deri.setup(20, 2.7);
    }

    void cal_single_axis_ctrl_input(double measure_single_axis, double loss_or_not, bool use_bias, int which_axis)
    {
        loss_or_not_ = loss_or_not;
        use_bias_ = use_bias;
        which_axis_ = which_axis;
        y_real = measure_single_axis;
        y_real = filter_for_img.filter(y_real);
        time_now = ros::Time::now().toSec();
        time_pass = time_now - time_last;
        y_real_derivative = (y_real - y_real_last) / time_pass; // 0.05 seconds = 50ms
        // y_filtered_deri = filter_for_deri.filter(y_real_derivative);
        y_filtered_deri = filter_4_for_deri.filter(y_real_derivative);
        function(loss_or_not_, use_bias_, which_axis_);
        y_real_last = y_real;
        time_last = time_now;
    }

    void function(double loss_or_not, bool use_bias, int which_axis)
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
            y_filtered_deri = 0;
            first_time_in_fun = false;
            pid2controller.computeControl(y_real, y_filtered_deri, u, integral_error, use_bias, which_axis);
        }
        else
        {
            pid2controller.computeControl(y_real, y_filtered_deri, u, integral_error, use_bias, which_axis);
        }
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        px4_state = msg->data;
        if (msg->data != 3) // 不在cmd模式下时，控制量为0；
        {
            u = 0;
            integral_error = 0;
        }
    }
};

class TripleAxisController
{
private:
    MyController controllerX, controllerY, controllerZ;
    ros::NodeHandle nh;
    ros::Subscriber sub, ground_truth_sub, ground_truth_second_sub, ground_truth_pose_sub;
    ros::Publisher pub_hat_x, acc_cmd_pub, pub_land, x_pub;
    quadrotor_msgs::PositionCommand acc_msg;
    double des_yaw;
    double ground_truth_first_deri_x = 0, ground_truth_first_deri_y = 0, ground_truth_first_deri_z = 0;
    double ground_truth_x = 0, ground_truth_y = 0, ground_truth_z = 0;
    double last_time = 0;
    bool keep_in_land = false;

public:
    // 构造函数
    TripleAxisController()
        : nh("~"), des_yaw(0)
    {
        x_pub = nh.advertise<std_msgs::Float64>("/input_x_axis", 100);
        sub = nh.subscribe("/point_with_fixed_delay", 1, &TripleAxisController::callback, this);
        ground_truth_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, &TripleAxisController::ground_truth_callback, this);
        ground_truth_pose_sub = nh.subscribe("/vrpn_client_node/MCServer/5/pose", 10, &TripleAxisController::ground_truth_pose_callback, this);
        pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        // 更新每个轴的控制器
        des_yaw = 0;

        controllerX.cal_single_axis_ctrl_input(msg->data[0], msg->data[4], 1, 0);
        controllerY.cal_single_axis_ctrl_input(msg->data[1], msg->data[4], 1, 1);
        controllerZ.cal_single_axis_ctrl_input(msg->data[2], msg->data[4], 1, 2); // 定高跟踪

        std_msgs::Float64 input_data_msg;
        input_data_msg.data = controllerX.u;
        x_pub.publish(input_data_msg);

        acc_msg.position.x = 0;
        acc_msg.position.y = 0;
        acc_msg.position.z = 0;
        acc_msg.velocity.x = 0;
        acc_msg.velocity.y = 0;
        acc_msg.velocity.z = 0;
        acc_msg.acceleration.x = controllerX.u;
        acc_msg.acceleration.y = controllerY.u;
        acc_msg.acceleration.z = controllerZ.u;
        acc_msg.jerk.x = 0;
        acc_msg.jerk.y = 0;
        acc_msg.jerk.z = 0;
        acc_msg.yaw = des_yaw;
        acc_msg.yaw_dot = 0;
        acc_msg.header.frame_id = 'world';
        acc_cmd_pub.publish(acc_msg);

        std_msgs::Float64MultiArray msg1;
        msg1.data.resize(27);

        msg1.data[2] = controllerX.u;
        msg1.data[3] = controllerX.y_real;
        msg1.data[4] = -ground_truth_first_deri_x;
        msg1.data[5] = ground_truth_first_deri_x;
        msg1.data[6] = controllerX.y_filtered_deri;
        msg1.data[7] = ground_truth_first_deri_x;
        msg1.data[8] = ground_truth_x;

        msg1.data[11] = controllerY.u;
        msg1.data[12] = controllerY.y_real;
        msg1.data[13] = -ground_truth_first_deri_y;
        msg1.data[14] = ground_truth_first_deri_y;
        msg1.data[15] = controllerY.y_filtered_deri;
        msg1.data[16] = ground_truth_first_deri_y;
        msg1.data[17] = ground_truth_y;

        msg1.data[20] = controllerZ.u;
        msg1.data[21] = controllerZ.y_real;
        msg1.data[22] = -ground_truth_first_deri_z;
        msg1.data[23] = ground_truth_first_deri_z;
        msg1.data[24] = controllerZ.y_filtered_deri;
        msg1.data[25] = ground_truth_first_deri_z;
        msg1.data[26] = ground_truth_z;
        pub_hat_x.publish(msg1);
    }

    void ground_truth_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        ground_truth_first_deri_x = msg->twist.linear.x;
        ground_truth_first_deri_y = msg->twist.linear.y;
        ground_truth_first_deri_z = msg->twist.linear.z;
    }

    void ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ground_truth_x = msg->pose.position.x;
        ground_truth_y = msg->pose.position.y;
        ground_truth_z = msg->pose.position.z;
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
    TripleAxisController controller;
    // TODO: Add ROS subscribers, publishers, and service servers/clients as needed
    controller.spin();
    return 0;
}
