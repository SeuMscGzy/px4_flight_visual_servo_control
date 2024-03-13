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

using namespace std;

class AIC3Controller
{
private:
    const double e_1 = 4.0;
    const double e_2 = 4.0;
    const double e_3 = 4.0;
    const double k_l = 1.0;
    const double sigma_z1_inv = 0.2;
    const double sigma_z2_inv = 0.2;
    const double sigma_z3_inv = 0.2;
    const double sigma_w1_inv = 1.0;
    const double sigma_w2_inv = 1.0;
    const double sigma_w3_inv = 1.0;
    const double k_i = -1;
    const double k_p = -3;
    const double k_d = -2.5; // 逐渐增大
    const double T_c = 0.01;
    const double x_bias = -0.7;
    const double y_bias = -0.2;
    const double z_bias = 0.3;
    double y1_APO_fast_bias = 0;
    double y1_real_bias = 0;
    std::vector<double> trust_array_y1 = {1, 0.75, 0.5, 0.25, 0};
    std::vector<double> trust_array_y2 = {0.7, 0.7, 0.7, 0.7, 0.7};
    std::vector<double> trust_array_y3 = {0.5, 0.5, 0.5, 0.5, 0.5};

public:
    void computeControl(int time_count, bool use_y1_real, double y1_real_slow, double y1_APO_fast, double y2_derivative_sampling, double y2_APO_fast, double y3_real_slow, double y3_APO_fast, double mu_last, double mu_p_last, double mu_pp_last, double u_last, std::atomic<double> &u, double &delta_u, double &mu, double &mu_p, double &mu_pp, bool use_bias, int which_axis)
    {
        if (!use_y1_real)
        {
            if (use_bias)
            {
                if (which_axis == 0)
                {
                    y1_APO_fast_bias = y1_APO_fast + x_bias;
                }
                else if (which_axis == 1)
                {
                    y1_APO_fast_bias = y1_APO_fast + y_bias;
                }
                else
                {
                    y1_APO_fast_bias = y1_APO_fast + z_bias;
                }
            }
            else
            {
                y1_APO_fast_bias = y1_APO_fast;
            }
            mu = double(mu_last + T_c * (mu_p_last + k_l * sigma_z1_inv * (y1_APO_fast_bias - mu_last) - k_l * sigma_w1_inv * e_1 * (mu_p_last + e_1 * mu_last)));
            mu_p = double(mu_p_last + T_c * (mu_pp_last + k_l * sigma_z2_inv * (y2_APO_fast - mu_p_last) - k_l * sigma_w1_inv * (mu_p_last + e_1 * mu_last) - k_l * sigma_w2_inv * e_2 * (mu_pp_last + e_2 * mu_p_last)));
            mu_pp = double(mu_pp_last + T_c * (k_l * sigma_z3_inv * (y3_real_slow - mu_pp_last) - k_l * sigma_w2_inv * (mu_pp_last + e_2 * mu_p_last) - k_l * sigma_w3_inv * e_3 * e_3 * mu_pp_last));
            delta_u = T_c * (k_i * (y1_APO_fast_bias - mu) + k_p * (y2_APO_fast - mu_p) + k_d * (y3_real_slow - mu_pp));
            u = double(u_last - delta_u);
        }
        else
        {
            if (use_bias)
            {
                if (which_axis == 0)
                {
                    y1_APO_fast_bias = y1_APO_fast + x_bias;
                    y1_real_bias = y1_real_slow + x_bias;
                }
                else if (which_axis == 1)
                {
                    y1_APO_fast_bias = y1_APO_fast + y_bias;
                    y1_real_bias = y1_real_slow + y_bias;
                }
                else
                {
                    y1_APO_fast_bias = y1_APO_fast + z_bias;
                    y1_real_bias = y1_real_slow + z_bias;
                }
            }
            else
            {
                y1_APO_fast_bias = y1_APO_fast;
                y1_real_bias = y1_real_slow;
            }
            double trust_param_y1 = trust_array_y1[time_count];
            double trust_param_y2 = trust_array_y2[time_count];
            double trust_param_y3 = trust_array_y3[time_count];
            mu = double(mu_last + T_c * (mu_p_last + (1 - trust_param_y1) * k_l * sigma_z1_inv * (y1_APO_fast_bias - mu_last) + trust_param_y1 * k_l * sigma_z1_inv * (y1_real_bias - mu_last) - k_l * sigma_w1_inv * e_1 * (mu_p_last + e_1 * mu_last)));
            mu_p = double(mu_p_last + T_c * ((1 - trust_param_y2) * k_l * sigma_z2_inv * (y2_APO_fast - mu_p_last) + trust_param_y2 * k_l * sigma_z2_inv * (y2_derivative_sampling - mu_p_last) - k_l * sigma_w1_inv * (mu_p_last + e_1 * mu_last) - k_l * sigma_w2_inv * e_2 * (mu_pp_last + e_2 * mu_p_last)));
            mu_pp = double(mu_pp_last + T_c * ((1 - trust_param_y3) * k_l * sigma_z3_inv * (y3_APO_fast - mu_pp_last) + trust_param_y3 * k_l * sigma_z3_inv * (y3_real_slow - mu_pp_last) - k_l * sigma_w2_inv * (mu_pp_last + e_2 * mu_p_last) - k_l * sigma_w3_inv * e_3 * e_3 * mu_pp_last));
            delta_u = T_c * (k_i * ((1 - trust_param_y1) * (y1_APO_fast_bias - mu) + trust_param_y1 * (y1_real_bias - mu)) + k_p * ((1 - trust_param_y2) * (y2_APO_fast - mu_p) + trust_param_y2 * (y2_derivative_sampling - mu_p)) + k_d * ((1 - trust_param_y3) * (y3_APO_fast - mu_pp) + trust_param_y3 * (y3_real_slow - mu_pp)));
            u = double(u_last - delta_u);
        }
        if (abs(u) >= 0.3) // 控制量限幅
        {
            u = 0.3 * u / abs(u);
        }
    }
};

/*class SampledDataController
{
private:
    const double k_p = 12.0;
    const double k_d = 6.0;

public:
    void computeControl(double y1_APO_fast, double y2_APO_fast, double disturbance_observe, std::atomic<double> &u)
    {
        u = -k_p * y1_APO_fast - k_d * y2_APO_fast - disturbance_observe;
    }
};*/

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
        a0 = 1.0 / (1.0 + q * ita + ita * ita);
        a1 = 2 * a0;
        a2 = a0;
        b1 = 2.0 * a0 * (1 - ita * ita);
        b2 = a0 * (1 - q * ita + ita * ita);
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
    Eigen::Vector3d hat_x_last, hat_x, B_bar, C_bar, B0;
    std::atomic<double> u;
    double predict_y, predict_y_last, y_real, u_last, delta_u, delta_u_last, t_now, measure;
    Eigen::Matrix3d A_bar, A0;
    double y_real_last, y_real_derivative_last;
    double y_real_derivative, y_real_second_derivative, y_filtered_deri, y_filtered_2deri;
    double mu, mu_p, mu_pp, mu_last, mu_p_last, mu_pp_last;
    int count;
    bool first_time_in_fun, first_time_cal_2deri;
    ros::Timer timer;
    int timer_count;
    ros::NodeHandle nh;
    double time_now, time_last, time_pass;
    // LowPassFilter filter_for_img, filter_for_deri, filter_for_2deri; // 一阶LPF
    ButterworthLowPassFilter filter_for_img, filter_for_deri, filter_for_2deri; // 二阶巴特沃斯LPF
    AIC3Controller aic3controller;
    ros::Subscriber px4_state_sub;
    friend class TripleAxisController;
    bool loss_target;
    double loss_or_not_;
    bool use_bias_;
    int which_axis_;

public:
    // 构造函数
    MyController()
        : hat_x_last(Eigen::Vector3d::Zero()),
          nh("~"),
          hat_x(Eigen::Vector3d::Zero()),
          predict_y_last(0.0),
          predict_y(0.0),
          y_real(0.0),
          measure(0.0),
          u(0.0),
          u_last(0.0),
          delta_u(0.0),
          delta_u_last(0.0),
          t_now(0.0),
          y_real_last(0.0),
          y_real_derivative_last(0.0),
          y_real_derivative(0.0),
          y_real_second_derivative(0.0),
          time_now(0.0),
          time_last(0.0),
          time_pass(0.0),
          mu(0.0),
          mu_p(0.0),
          mu_pp(0.0),
          mu_last(0.0),
          mu_p_last(0.0),
          mu_pp_last(0.0),
          timer_count(0),
          // filter_for_img(0.84),  // 0.8 is well
          // filter_for_deri(0.35), // 0.41 is well
          // filter_for_2deri(0.12),
          filter_for_img(20, 9.8),
          filter_for_deri(20, 2.5),
          filter_for_2deri(20, 0.9),
          y_filtered_deri(0),
          y_filtered_2deri(0),
          loss_target(true),
          loss_or_not_(1),
          use_bias_(1),
          which_axis_(0),
          first_time_in_fun(true),
          first_time_cal_2deri(false)
    {
        A_bar << 0.641233388759048, 0.00812240265642060, 4.35985113066055e-05,
            -4.68558842659452, 0.975064137937935, 0.00991430147112208,
            -20.8855644377951, -0.112107162843774, 0.999613151699345;
        B_bar << 1.37568842851619e-07,
            4.97433857418070e-05,
            0.00999678544531325;
        C_bar << 0.358766611241019,
            4.68558842659614,
            20.8855644378049;
        /*A_bar << 0.704170422049836, 0.00846563257855219, 4.47917067648264e-05,
            -3.13264238771843, 0.983536297142058, 0.00994375890179147,
            -11.2677569620530, -0.0596177617039839, 0.999795686697690;
        B_bar << 1.37568842851619e-07,
            4.97433857418070e-05,
            0.00999678544531325;
        C_bar << 0.295829577950179,
            3.13264238771871,
            11.2677569620544;*/
        /* A_bar << 0.634457640194048, 0.00808503158920889, 4.34679117699403e-05,
             -4.87327452435155, 0.974028966940822, 0.00991068388354639,
             -22.1853266807892, -0.119275949896716, 0.999588099061547;
         B_bar << 1.37568842851619e-07,
             4.97433857418070e-05,
             0.00999678544531325;
         C_bar << 0.365542359806030,
             4.87327452435347,
             22.1853266808012;*/
        A0 << 1, 0.0100000000000000, 5.00000000000000e-05,
            0, 1, 0.0100000000000000,
            0, 0, 1;
        B0 << 1.66666666666667e-07,
            5.00000000000000e-05,
            0.0100000000000000;
        px4_state_sub = nh.subscribe("/px4_state_pub", 1, &MyController::StateCallback, this);
        /*XmlRpc::XmlRpcValue params;
        params["order"] = 2;
        params["cutoff_frequency"] = 20.0; // 输出测量的截止频率
        filter_for_img_.configure(params, nh);
        params["cutoff_frequency"] = 10.0; // 输出测量一阶差分的截止频率
        filter_for_deri_.configure(params, nh);
        params["cutoff_frequency"] = 5.0; // 输出测量一阶差分的截止频率
        filter_for_2deri_.configure(params, nh);*/
    }

    void cal_single_axis_ctrl_input(double measure_single_axis, double loss_or_not, bool use_bias, int which_axis)
    {
        loss_or_not_ = loss_or_not;
        use_bias_ = use_bias;
        which_axis_ = which_axis;
        measure = measure_single_axis;
        y_real = measure;
        y_real = filter_for_img.filter(y_real);
        // filter_for_img_.update(measure, y_real);

        time_now = ros::Time::now().toSec();
        time_pass = time_now - time_last;
        function(loss_or_not_, use_bias_, which_axis_);

        y_real_derivative = (y_real - y_real_last) / time_pass; // 0.05 seconds = 50ms
        y_filtered_deri = filter_for_deri.filter(y_real_derivative);
        // filter_for_deri_.update(y_real_derivative, y_filtered_deri);

        if (first_time_cal_2deri)
        {
            y_real_derivative_last = y_real_derivative;
            first_time_cal_2deri = false;
        }

        // Compute the second derivative
        y_real_second_derivative = (y_real_derivative - y_real_derivative_last) / time_pass;
        y_filtered_2deri = filter_for_2deri.filter(y_real_second_derivative);
        // filter_for_2deri_.update(y_real_second_derivative, y_filtered_2deri);

        // Update the last values for the next iteration
        y_real_derivative_last = y_real_derivative;
        y_real_last = y_real;
        timer_count++;
        timer = nh.createTimer(ros::Duration(0.01), &MyController::timerCallback, this);
        time_last = time_now;
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
            first_time_cal_2deri = true;
            time_pass = 0.05;
            y_real_last = y_real;
            first_time_in_fun = false;
            // hat_x = Eigen::Vector3d::Zero();
            predict_y = y_real;
            hat_x(0) = y_real;
            aic3controller.computeControl(timer_count, 1, y_real, hat_x(0), y_filtered_deri, hat_x(1), y_filtered_2deri, hat_x(2), mu_last, mu_p_last, mu_pp_last, u_last, u, delta_u, mu, mu_p, mu_pp, use_bias, which_axis);
            // sampleddatacontroller.computeControl(hat_x(0), hat_x(1), hat_x(2), u);
            // u = 0;
            // delta_u = 0;
        }
        else
        {
            if (timer_count == 0)
            {
                predict_y = y_real;
                hat_x = A_bar * hat_x_last + B_bar * delta_u_last + C_bar * predict_y;
                aic3controller.computeControl(timer_count, 1, y_real, hat_x(0), y_filtered_deri, hat_x(1), y_filtered_2deri, hat_x(2), mu_last, mu_p_last, mu_pp_last, u_last, u, delta_u, mu, mu_p, mu_pp, use_bias, which_axis);
                // sampleddatacontroller.computeControl(hat_x(0), hat_x(1), hat_x(2), u);
                // u = 0;
                // delta_u = 0;
            }
            else
            {
                Eigen::Vector3d coeff(1, 0, 0);
                predict_y = coeff.transpose() * (A0 * hat_x_last + B0 * delta_u_last);
                hat_x = A_bar * hat_x_last + B_bar * delta_u_last + C_bar * predict_y_last;
                aic3controller.computeControl(timer_count, 1, y_real, hat_x(0), y_filtered_deri, hat_x(1), y_filtered_2deri, hat_x(2), mu_last, mu_p_last, mu_pp_last, u_last, u, delta_u, mu, mu_p, mu_pp, use_bias, which_axis);
                // sampleddatacontroller.computeControl(hat_x(0), hat_x(1), hat_x(2), u);
                // u = 0;
                // delta_u = 0;
            }
        }
        // Update last values for the next iteration
        mu_last = mu;
        mu_p_last = mu_p;
        mu_pp_last = mu_pp;
        hat_x_last = hat_x;
        predict_y_last = predict_y;
        u_last = u;
        // delta_u_last = delta_u;
        delta_u_last = 0;
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (msg->data != 3) // 不在cmd模式下时，控制量为0；
        {
            u = 0;
            u_last = 0;
            delta_u = 0;
            delta_u_last = 0;
            mu = 0;
            mu_p = 0;
            mu_pp = 0;
            mu_last = 0;
            mu_p_last = 0;
            mu_pp_last = 0;
        }
    }
};

class TripleAxisController
{
private:
    MyController controllerX, controllerY, controllerZ;
    ros::Subscriber sub, ground_truth_sub, ground_truth_second_sub;
    ros::Publisher pub_hat_x, acc_cmd_pub;
    ros::NodeHandle nh;
    quadrotor_msgs::PositionCommand acc_msg;
    ros::Publisher land_pub;
    ros::Timer control_update_timer;
    double ground_truth_first_deri_x = 0, ground_truth_second_deri_x = 0;

public:
    // 构造函数
    TripleAxisController()
        : nh("~")
    {
        sub = nh.subscribe("/delay_sent_50ms", 1, &TripleAxisController::callback, this);
        ground_truth_sub = nh.subscribe("/vrpn_client_node/MCServer/5/velocity", 1, &TripleAxisController::ground_truth_callback, this);
        ground_truth_second_sub = nh.subscribe("/vrpn_client_node/MCServer/5/acceleration", 1, &TripleAxisController::ground_truth_second_callback, this);
        pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 100);
        land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 100);
        control_update_timer = nh.createTimer(ros::Duration(0.01), &TripleAxisController::controlUpdate, this);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        // 更新每个轴的控制器
        // if (abs(msg->data[0]) > 0.2 || abs(msg->data[1]) > 0.2)
        //{
        controllerX.cal_single_axis_ctrl_input(msg->data[0], msg->data[4], 1, 0);
        controllerY.cal_single_axis_ctrl_input(msg->data[1], msg->data[4], 0, 1);
        controllerZ.cal_single_axis_ctrl_input(msg->data[2], msg->data[4], 1, 2); // 定高跟踪
        //}
        /*else if (abs(msg->data[0]) != 0 && abs(msg->data[1]) != 0) // 跟踪误差很小 可以开始边下降边跟踪
        {
            controllerX.cal_single_axis_ctrl_input(msg->data[0], msg->data[4],0,0);
            controllerY.cal_single_axis_ctrl_input(msg->data[1], msg->data[4],0,1);
            // controllerZ.first_time_in_fun = true; //这样每次都会赋值，考虑加入逻辑判断。
            controllerZ.cal_single_axis_ctrl_input(msg->data[2], msg->data[4],0,2);                // 下降
            if (abs(msg->data[0]) < 0.2 && abs(msg->data[1]) < 0.2 && abs(msg->data[2]) < 0.3) // 着陆
            {
                quadrotor_msgs::TakeoffLand msg;
                msg.takeoff_land_cmd = 2; // 设置为着陆指令
                land_pub.publish(msg);
            }
        }*/
    }

    void ground_truth_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        ground_truth_first_deri_x = msg->twist.linear.x;
    }

    void ground_truth_second_callback(const geometry_msgs::AccelStamped::ConstPtr &msg)
    {
        ground_truth_second_deri_x = msg->accel.linear.x;
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
        acc_msg.yaw = 0;
        acc_msg.header.frame_id = 'world';
        acc_cmd_pub.publish(acc_msg);
        std_msgs::Float64MultiArray msg1;
        msg1.data.resize(9);
        for (int i = 0; i < 3; i++)
        {
            msg1.data[i] = controllerX.hat_x(i);
        }
        msg1.data[3] = controllerX.y_real;
        // msg1.data[4] = controllerX.y_real_derivative;
        msg1.data[4] = -ground_truth_first_deri_x;
        msg1.data[5] = -ground_truth_second_deri_x;
        msg1.data[6] = controllerX.y_filtered_deri;
        msg1.data[7] = controllerX.y_filtered_2deri;
        msg1.data[8] = controllerX.measure;
        // cout << "消息已经发送" << endl;
        pub_hat_x.publish(msg1);
    }

    void spin()
    {
        ros::Rate rate(500);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

    // 其他必要的方法
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_controller_node");
    TripleAxisController controller;
    // TODO: Add ROS subscribers, publishers, and service servers/clients as needed
    controller.spin();
    return 0;
}
