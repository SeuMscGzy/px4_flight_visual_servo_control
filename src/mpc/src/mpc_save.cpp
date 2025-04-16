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
using namespace ACADO;

/*class MPCController
{
private:
    double x_bias = 0;
    double y_bias = 0;
    double z_bias = 0;
    Eigen::Vector3d xyz_1_bias = Eigen::Vector3d::Zero();

    // ACADO 相关对象
    DifferentialState x1, x2, y1, y2, z1, z2; // 系统状态：x1（位置），x2（速度）
    Control ux, uy, uz;                       // 控制输入
    RealTimeAlgorithm *alg;                   // 实时求解算法
    ACADO::Controller *mpc_controller;        // MPC 控制器求解器
    DVector current_state;                    // 当前状态，维度为 6
    const double u_max = 4.0;                 // 控制输入幅值限制
    // MPC 参数 (先声明常量，再声明依赖的变量！)
    const double T = 0.75;
    const int N = 15;
    double dt;

    // 然后再定义 acado 对象
    DifferentialEquation f;
    OCP ocp;

public:
    MPCController() : ocp(0.0, T, N), current_state(6)
    {
        // 计算采样时间
        dt = T / N;

        // 构造系统动态方程：x1_dot = x2,  x2_dot = u
        f << dot(x1) == x2;
        f << dot(x2) == ux;
        f << dot(y1) == y2;
        f << dot(y2) == uy;
        f << dot(z1) == z2;
        f << dot(z2) == uz;

        // 设置最优控制问题 OCP
        ocp.subjectTo(f);
        // 添加控制输入约束：u ∈ [-u_max, u_max]
        ocp.subjectTo(-u_max <= ux <= u_max);
        ocp.subjectTo(-u_max <= uy <= u_max);
        ocp.subjectTo(-u_max <= uz <= u_max);

        // 构造测量向量 h = [x1, x2, u]，用于最小二乘成本
        Function h;
        h << x1;
        h << x2;
        h << y1;
        h << y2;
        h << z1;
        h << z2;
        h << ux;
        h << uy;
        h << uz;

        // 参考向量r是零向量
        DVector ref(9);
        ref.setZero();

        // 定义加权矩阵，对状态给予单位权重，对控制输入权重较小（0.1）
        DMatrix W(9, 9);
        W.setIdentity();
        W(0, 0) = 3; // x1
        W(1, 1) = 1; // x2
        W(2, 2) = 3; // y1
        W(3, 3) = 1; // y2
        W(4, 4) = 3; // z1
        W(5, 5) = 1; // z2
        W(6, 6) = 0.3;
        W(7, 7) = 0.3;
        W(8, 8) = 0.3;
        ocp.minimizeLSQ(W, h, ref);

        // 配置实时算法参数
        alg = new RealTimeAlgorithm(ocp, dt);
        alg->set(INTEGRATOR_TYPE, INT_RK4);
        alg->set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        alg->set(PRINTLEVEL, NONE);

        // 创建 Controller 对象
        mpc_controller = new Controller(*alg);

        ROS_INFO("MPC start");
    }

    // 析构函数，释放 ACADO 资源
    ~MPCController()
    {
        if (mpc_controller != nullptr)
            delete mpc_controller;
        if (alg != nullptr)
            delete alg;
    }

    void adjustBias(Eigen::Vector3d xyz_1) // which_axis: 0 for x, 1 for y, 2 for z
    {
        xyz_1_bias = xyz_1 + Eigen::Vector3d(x_bias, y_bias, z_bias);
    }

    Eigen::Vector3d computeControl(Eigen::Vector3d xyz_1, Eigen::Vector3d xyz_2)
    {
        adjustBias(xyz_1);
        current_state(0) = xyz_1_bias(0);
        current_state(1) = xyz_2(0);
        current_state(2) = xyz_1_bias(1);
        current_state(3) = xyz_2(1);
        current_state(4) = xyz_1_bias(2);
        current_state(5) = xyz_2(2);

        static bool is_initialized = false;
        double currentTime = ros::Time::now().toSec();

        if (!is_initialized)
        {
            if (mpc_controller->init(currentTime, current_state) != SUCCESSFUL_RETURN)
            {
                ROS_ERROR("MPC initialization failed!");
                Eigen::Vector3d u;
                u.setZero();
                return u;
            }
            is_initialized = true;
            ROS_INFO("MPC initialized successfully.");
        }
        // Start timing
        ros::Time start_time = ros::Time::now();
        // 调用 MPC 求解器

        if (mpc_controller->step(currentTime, current_state) != SUCCESSFUL_RETURN)
        {
            ros::Time end_time = ros::Time::now();
            double elapsed_time = (end_time - start_time).toSec();
            ROS_ERROR("MPC step failed! Elapsed time: %f seconds", elapsed_time);
            ROS_ERROR("MPC failed!");
            Eigen::Vector3d u;
            u.setZero();
            return u;
        }
        ros::Time end_time = ros::Time::now();
        double elapsed_time = (end_time - start_time).toSec();
        ROS_INFO("MPC Elapsed time: %f seconds", elapsed_time);
        DVector u_(3);
        if (mpc_controller->getU(u_) == SUCCESSFUL_RETURN)
        {
            Eigen::Vector3d u;
            u << u_(0), u_(1), u_(2);
            return u;
        }
        else
        {
            ROS_ERROR("Failed to retrieve control input from MPC.");
            Eigen::Vector3d u;
            u.setZero();
            return u;
        }
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
    ros::Subscriber px4_state_sub;
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
        filter_4_for_derix.setup(20, 2.7);
        filter_4_for_deriy.setup(20, 2.7);
        filter_4_for_deriz.setup(20, 2.7);
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
};

class TripleAxisController
{
private:
    ros::NodeHandle nh;
    MyController my_controller;
    ros::Subscriber sub, ground_truth_sub, ground_truth_second_sub, ground_truth_pose_sub;
    ros::Publisher pub_hat_x, acc_cmd_pub, x_pub;
    quadrotor_msgs::PositionCommand acc_msg;
    double des_yaw;
    double ground_truth_first_deri_x = 0, ground_truth_first_deri_y = 0, ground_truth_first_deri_z = 0;
    double ground_truth_x = 0, ground_truth_y = 0, ground_truth_z = 0;
    double last_time = 0;

public:
    // 构造函数
    TripleAxisController()
        : nh(), des_yaw(0)
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
        des_yaw = 0;
        Eigen::Vector3d xyz_1;
        xyz_1 << msg->data[0], msg->data[1], msg->data[2];
        my_controller.cal_single_axis_ctrl_input(xyz_1, msg->data[4]);
        acc_msg.position.x = 0;
        acc_msg.position.y = 0;
        acc_msg.position.z = 0;
        acc_msg.velocity.x = 0;
        acc_msg.velocity.y = 0;
        acc_msg.velocity.z = 0;
        acc_msg.acceleration.x = my_controller.u(0);
        acc_msg.acceleration.y = my_controller.u(1);
        acc_msg.acceleration.z = my_controller.u(2);
        acc_msg.jerk.x = 0;
        acc_msg.jerk.y = 0;
        acc_msg.jerk.z = 0;
        acc_msg.yaw = des_yaw;
        acc_msg.yaw_dot = 0;
        acc_msg.header.frame_id = 'world';
        acc_cmd_pub.publish(acc_msg);

        std_msgs::Float64 input_data_msg;
        input_data_msg.data = my_controller.u(0);
        x_pub.publish(input_data_msg);

        std_msgs::Float64MultiArray msg1;
        msg1.data.resize(27);

        msg1.data[2] = my_controller.u(0);
        msg1.data[3] = my_controller.x_real;
        msg1.data[4] = -ground_truth_first_deri_x;
        msg1.data[5] = ground_truth_first_deri_x;
        msg1.data[6] = my_controller.x_filtered_deri;
        msg1.data[7] = ground_truth_first_deri_x;
        msg1.data[8] = ground_truth_x;

        msg1.data[11] = my_controller.u(1);
        msg1.data[12] = my_controller.y_real;
        msg1.data[13] = -ground_truth_first_deri_y;
        msg1.data[14] = ground_truth_first_deri_y;
        msg1.data[15] = my_controller.y_filtered_deri;
        msg1.data[16] = ground_truth_first_deri_y;
        msg1.data[17] = ground_truth_y;

        msg1.data[20] = my_controller.u(2);
        msg1.data[21] = my_controller.z_real;
        msg1.data[22] = -ground_truth_first_deri_z;
        msg1.data[23] = ground_truth_first_deri_z;
        msg1.data[24] = my_controller.z_filtered_deri;
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
