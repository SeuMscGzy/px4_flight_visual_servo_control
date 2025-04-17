#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
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
#include <std_msgs/Empty.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
using namespace std;

class PIDController
{
private:
    double k_i = 0.5;
    double k_p = 4;
    double k_d = 3;
    const double T_c = 0.01;
    double x_bias = -1;
    double y_bias = 0;
    double z_bias = -0.1;
    double y1_fast_bias = 0;
    double integral_error = 0;
    ros::NodeHandle nh;

public:
    PIDController() // 构造函数
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

    void setIntegralZero()
    {
        integral_error = 0;
    }

    double limitControl(double controlValue, int which_axis)
    {
        double limit = (which_axis != 0) ? 3 : 4; // Limit is 3 for y and z, 1.5 for x axis.
        if (abs(controlValue) >= limit)
        {
            controlValue = limit * controlValue / abs(controlValue); // Apply limit
        }
        return controlValue;
    }

    double limitIntegral(double IntegralValue, int which_axis)
    {
        double limit = (which_axis != 0) ? 1 : 1; // Limit is 1.5 for y and z, 0.75 for x axis.
        if (abs(IntegralValue) >= limit)
        {
            IntegralValue = limit * IntegralValue / abs(IntegralValue); // Apply limit
        }
        return IntegralValue;
    }

    void computeControl(double y1_fast, double y2_fast, bool use_bias, int which_axis, double &u)
    {
        y1_fast_bias = adjustBias(y1_fast, which_axis, use_bias);
        integral_error = integral_error + y1_fast_bias * T_c;
        integral_error = limitIntegral(integral_error, which_axis);
        u = k_i * integral_error + k_p * y1_fast_bias + k_d * y2_fast;
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

class MyController
{
private:
    Eigen::Vector2d hat_x_last, hat_x, B_bar, C_bar, B0;
    Eigen::Matrix2d A_bar, A0;
    double u;
    double u_last;
    double y_real;
    int timer_count;
    bool first_time_in_fun, loss_target, use_bias_;
    ros::NodeHandle nh;
    ros::Timer timer;

    LowPassFilter filter_for_img;
    PIDController pidcontroller;
    ros::Subscriber px4_state_sub;
    friend class TripleAxisController;
    double loss_or_not_;
    int which_axis_;
    int px4_state;

public:
    // 构造函数
    MyController()
        : hat_x_last(Eigen::Vector2d::Zero()),
          nh("~"),
          px4_state(1),
          hat_x(Eigen::Vector2d::Zero()),
          y_real(0.0),
          u(0.0),
          u_last(0.0),
          timer_count(0),
          filter_for_img(0.95),
          loss_target(true),
          loss_or_not_(1),
          use_bias_(1),
          which_axis_(0),
          first_time_in_fun(true)
    {
        A_bar << 0.563905116202294, 0.00767209146671299,
            -5.38580820963311, 0.970525963938104;
        B_bar << 4.48259305755374e-05,
            0.00995822807882039;
        C_bar << 0.436094883797706,
            5.38580820963311;
        A0 << 1, 0.0100000000000000,
            0, 1;
        B0 << 5.00000000000000e-05,
            0.0100000000000000;
        px4_state_sub = nh.subscribe("/px4_state_pub", 1, &MyController::StateCallback, this, ros::TransportHints().tcpNoDelay());
    }

    void cal_single_axis_ctrl_input(double measure_single_axis, double loss_or_not, bool use_bias, int which_axis)
    {
        loss_or_not_ = loss_or_not;
        use_bias_ = use_bias;
        which_axis_ = which_axis;
        y_real = measure_single_axis;
        y_real = filter_for_img.filter(y_real);
        function(loss_or_not_, use_bias_, which_axis_);
        // Update the last values for the next iteration
        timer_count++;
        timer = nh.createTimer(ros::Duration(0.01), &MyController::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        function(loss_or_not_, use_bias_, which_axis_);
        timer_count++;
        if (timer_count >= 5)
        {
            timer.stop();
            timer_count = 0;
        }
    }

    void function(double loss_or_not, bool use_bias, int which_axis)
    {
        if (loss_or_not == 1 && loss_target == false) // From seeing the target to not seeing the target
        {
            loss_target = true;
        }
        if (loss_or_not == 0 && loss_target == true) // From not seeing the target to seeing the target
        {
            loss_target = false;
            first_time_in_fun = true;
        }
        if (first_time_in_fun)
        {
            first_time_in_fun = false;
            hat_x(0) = y_real;
            hat_x(1) = 0;
            pidcontroller.setIntegralZero();
            pidcontroller.computeControl(hat_x(0), hat_x(1), use_bias, which_axis, u);
        }
        else
        {
            if (timer_count == 0)
            {
                u_last = 0;
                hat_x = A_bar * hat_x_last + B0 * u_last + C_bar * y_real;
                pidcontroller.computeControl(hat_x(0), hat_x(1), use_bias, which_axis, u);
            }
            else
            {
                u_last = 0;
                hat_x = A0 * hat_x_last + B0 * u_last;
                pidcontroller.computeControl(hat_x(0), hat_x(1), use_bias, which_axis, u);
            }
        }
        // Update last values for the next iteration
        hat_x_last = hat_x;
        u_last = u;
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (msg->data != 3) // When not in cmd mode, the control amount is 0.
        {
            pidcontroller.setIntegralZero();
            u = 0;
            u_last = 0;
        }
    }
};

class TripleAxisController
{
private:
    std::ofstream output_file; // 用于写入数据的文件
    int iterations;            // 迭代次数
    MyController controllerX, controllerY, controllerZ;
    ros::NodeHandle nh;
    ros::Subscriber sub, ground_truth_sub, ground_truth_pose_sub;
    ros::Publisher pub_hat_x, acc_cmd_pub;
    quadrotor_msgs::PositionCommand acc_msg;
    double des_yaw;
    ros::Timer control_update_timer;
    double ground_truth_first_deri_x = 0, ground_truth_first_deri_y = 0, ground_truth_first_deri_z = 0;
    double ground_truth_x = 0, ground_truth_y = 0, ground_truth_z = 0;

public:
    TripleAxisController()
        : nh("~"), des_yaw(0), iterations(0)
    {
        output_file.open("execution_times.csv");
        sub = nh.subscribe("/point_with_fixed_delay", 1, &TripleAxisController::callback, this, ros::TransportHints().tcpNoDelay());
        ground_truth_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, &TripleAxisController::ground_truth_callback, this);
        ground_truth_pose_sub = nh.subscribe("/vrpn_client_node/MCServer/5/pose", 10, &TripleAxisController::ground_truth_pose_callback, this);
        pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
        control_update_timer = nh.createTimer(ros::Duration(0.01), &TripleAxisController::controlUpdate, this);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        // Update the controller for each axis
        des_yaw = msg->data[5];
        std::clock_t start = std::clock();
        controllerX.cal_single_axis_ctrl_input(msg->data[0], msg->data[4], 1, 0);
        controllerY.cal_single_axis_ctrl_input(msg->data[1], msg->data[4], 0, 1);
        controllerZ.cal_single_axis_ctrl_input(msg->data[2], msg->data[4], 1, 2);
        std::clock_t end = std::clock();
        //  计算并输出执行时间（以毫秒为单位）
        double duration = 1000 * double(end - start) / CLOCKS_PER_SEC;
        cout << iterations << endl;
        if (iterations == 0)
        {
            output_file << "Iterations,Execution Time (ms)\n"; // 写入表头
        }
        else if (iterations <= 5000)
        {
            output_file << iterations << "," << duration << "\n";
        }
        else
        {
            output_file.close(); // 关闭文件
        }
        iterations++;
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

    void controlUpdate(const ros::TimerEvent &)
    {
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

        for (int i = 0; i < 2; i++)
        {
            msg1.data[i] = controllerX.hat_x(i);
        }
        for (int i = 9; i < 11; i++)
        {
            msg1.data[i] = controllerY.hat_x(i - 9);
        }
        for (int i = 18; i < 20; i++)
        {
            msg1.data[i] = controllerZ.hat_x(i - 18);
        }
        msg1.data[2] = controllerX.u;
        msg1.data[3] = controllerX.y_real;
        msg1.data[4] = -ground_truth_first_deri_x;
        msg1.data[5] = ground_truth_first_deri_x;
        msg1.data[6] = 0;
        msg1.data[7] = ground_truth_first_deri_x;
        msg1.data[8] = ground_truth_x;

        msg1.data[11] = controllerY.u;
        msg1.data[12] = controllerY.y_real;
        msg1.data[13] = -ground_truth_first_deri_y;
        msg1.data[14] = ground_truth_first_deri_y;
        msg1.data[15] = 0;
        msg1.data[16] = ground_truth_first_deri_y;
        msg1.data[17] = ground_truth_y;

        msg1.data[20] = controllerZ.u;
        msg1.data[21] = controllerZ.y_real;
        msg1.data[22] = -ground_truth_first_deri_z;
        msg1.data[23] = ground_truth_first_deri_z;
        msg1.data[24] = 0;
        msg1.data[25] = ground_truth_first_deri_z;
        msg1.data[26] = ground_truth_z;
        pub_hat_x.publish(msg1);
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