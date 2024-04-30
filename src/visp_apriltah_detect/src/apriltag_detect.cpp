#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
using namespace std;
bool use_forward_cam = false;
double data1[] = {0.003998954650746039, -0.03120742319497377, 0.9995049300024641,
                  -0.9999629576230795, 0.0074933480375017625, 0.0042347511015314865, -0.0076217939754581465, -0.9994848405417254, -0.031176301638856818};
double data2[] = {0.006888006690856245, -0.9995473067077326, -0.029287147644378266,
                  -0.9999346091207707, -0.0071521307333508215, 0.008923256606383264, -0.00912868261673192, 0.029223769080834707, -0.9995312093548614};
tf::Matrix3x3 convertCvMatToTfMatrix(const cv::Mat &cv_matrix)
{
    tf::Matrix3x3 tf_matrix;
    if (cv_matrix.rows == 3 && cv_matrix.cols == 3 && cv_matrix.type() == CV_64F)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                tf_matrix[i][j] = cv_matrix.at<double>(i, j);
            }
        }
    }
    else
    {
        std::cerr << "Invalid matrix size or type" << std::endl;
    }
    return tf_matrix;
}
double fromQuaternion2yaw(Eigen::Quaterniond q)
{
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}
class AprilTagDetector
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    std_msgs::Float64MultiArray point_;
    ros::Subscriber odom_sub_;
    ros::Publisher point_pub_;
    ros::Subscriber R_subscriber;
    double image_time;
    cv::Mat Position_before = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Position_after = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    tf::Matrix3x3 R_c2a;
    double desired_yaw;

public:
    AprilTagDetector()
    {
        desired_yaw = 0;
        cv::Mat tempMat(3, 3, CV_64F);
        if (use_forward_cam)
        {
            tempMat = cv::Mat(3, 3, CV_64F, data1);
        }
        else
        {
            tempMat = cv::Mat(3, 3, CV_64F, data2);
        }
        R = tempMat;
        image_sub_ = nh_.subscribe("/object_pose", 1, &AprilTagDetector::imageCb, this);
        odom_sub_ = nh_.subscribe("/vins_fusion/imu_propagate", 1, &AprilTagDetector::odomCallback, this);
        point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1);
        R_subscriber = nh_.subscribe("/R_data", 1, &AprilTagDetector::RCallback, this);
    }

    ~AprilTagDetector()
    {
    }

    void publishDetectionResult(bool lost)
    {
        // 简化为直接赋值
        point_.data = {Position_after.at<double>(0, 0), Position_after.at<double>(1, 0), Position_after.at<double>(2, 0), image_time, static_cast<double>(lost), desired_yaw};
        point_pub_.publish(point_);
        point_.data.clear();
    }

    void RCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() == 9)
        {
            // 更新类中的旋转矩阵
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R_c2a[i][j] = msg->data[i * 3 + j];
                }
            }
        }
        else
        {
            ROS_ERROR("Received matrix data does not have 9 elements.");
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        tf::Quaternion q1(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 rot_matrix1(q1); // R_W2i  世界系到imu
        tf::Matrix3x3 rot_matrix2(0.003998954650746039, -0.03120742319497377, 0.9995049300024641,
                                  -0.9999629576230795, 0.0074933480375017625, 0.0042347511015314865, -0.0076217939754581465, -0.9994848405417254, -0.031176301638856818); // 前部相机与imu的旋转关系
        tf::Matrix3x3 rot_matrix3(0.006888006690856245, -0.9995473067077326, -0.029287147644378266,
                                  -0.9999346091207707, -0.0071521307333508215, 0.008923256606383264, -0.00912868261673192, 0.029223769080834707, -0.9995312093548614); // Ric imu到相机
        tf::Matrix3x3 rot_matrix_cam_to_world;
        if (use_forward_cam)
        {
            rot_matrix_cam_to_world = rot_matrix1 * rot_matrix2;
        }
        else
        {
            rot_matrix_cam_to_world = rot_matrix1 * rot_matrix3;
        }

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.at<double>(i, j) = rot_matrix_cam_to_world[i][j];
            }
        }
    }

    void imageCb(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        std_msgs::Float64MultiArray temp_array;
        temp_array.data = msg->data;
        image_time = temp_array.data[4];
        // cout << "pos_recieve-img_recieve: " << 1000 * (ros::Time::now().toSec() - image_time) << " ms" << endl;
        if (temp_array.data[3] == 1)
        {
            if (use_forward_cam)
            {
                Position_before.at<double>(0, 0) = temp_array.data[0] + 0.00625206868300806;
                Position_before.at<double>(1, 0) = temp_array.data[1] - 0.08892133787128216;
                Position_before.at<double>(2, 0) = temp_array.data[2] + 0.0643024892575438; // 将其转换到imu飞控所在位置
                desired_yaw = 0;
            }
            else
            {
                Position_before.at<double>(0, 0) = temp_array.data[0] - 0.0040370827962106365;
                Position_before.at<double>(1, 0) = temp_array.data[1] - 0.06444548838690714;
                Position_before.at<double>(2, 0) = temp_array.data[2] + 0.008456530028234236; // 将其转换到imu飞控所在位置
                tf::Matrix3x3 R_tf = convertCvMatToTfMatrix(R);
                tf::Matrix3x3 R_W2a = R_tf * R_c2a;
                Eigen::Matrix3d eigen_mat;
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        eigen_mat(i, j) = R_W2a[i][j];
                    }
                }
                Eigen::Quaterniond q;
                q = Eigen::Quaterniond(eigen_mat);
                double yaw = fromQuaternion2yaw(q);
                yaw = yaw + M_PI / 2;
                cout << yaw << endl;
                desired_yaw = yaw;
            } // 由于相机安装角度问题，需要加上90度

            // cout << "Position: (" << Position_before.at<double>(0, 0) << ", " << Position_before.at<double>(1, 0) << ", " << Position_before.at<double>(2, 0) << ")" << endl;
            Position_after = R * Position_before;
            // cout << "Position: (" << Position_after.at<double>(0, 0) << ", " << Position_after.at<double>(1, 0) << ", " << Position_after.at<double>(2, 0) << ")" << endl;
            publishDetectionResult(false); // 提取的函数，用于发布检测结果
        }
        else
        {
            desired_yaw = 0;
            publishDetectionResult(true); // 提取的函数，用于发布检测结果
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_detector");
    AprilTagDetector atd;
    ros::spin();
    return 0;
}
