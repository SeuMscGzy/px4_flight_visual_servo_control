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
using namespace std;
double data1[] = {0.011763502382341945f, -0.03427874295798661f, 0.999343078123285f,
                  -0.999463220176014f, 0.030157061398510737f, 0.01279934229561297f,
                  -0.030575995929723154f, -0.9989572160152946f, -0.03390559015676936f};
class AprilTagDetector
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    std_msgs::Float64MultiArray point_;
    ros::Subscriber odom_sub_;
    ros::Publisher point_pub_;
    cv::Mat Position_before = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Position_after = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

public:
    AprilTagDetector()
    {
        cv::Mat tempMat(3, 3, CV_64F, data1);
        R = tempMat;
        image_sub_ = nh_.subscribe("/object_pose", 1, &AprilTagDetector::imageCb, this);
        odom_sub_ = nh_.subscribe("/vins_fusion/imu_propagate", 1, &AprilTagDetector::odomCallback, this);
        point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_unfixed_delay", 10);
    }

    ~AprilTagDetector()
    {
    }

    void publishDetectionResult(bool lost)
    {
        // 简化为直接赋值
        point_.data = {Position_after.at<double>(0, 0), Position_after.at<double>(1, 0), Position_after.at<double>(2, 0), ros::Time::now().toSec(), static_cast<double>(lost)};
        point_pub_.publish(point_);
        point_.data.clear();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        tf::Quaternion q1(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 rot_matrix1(q1);
        tf::Matrix3x3 rot_matrix2(0.011763502382341945, -0.03427874295798661, 0.999343078123285,
                                  -0.999463220176014, 0.030157061398510737, 0.01279934229561297, -0.030575995929723154, -0.9989572160152946, -0.03390559015676936);
        // 这是加入底部相机的旋转矩阵，两个相机之间朝向的粗标定。
        // tf::Matrix3x3 rot_matrix3(1, 0, 0,
        //                         0, 0, 1, 0, -1, 0);
        // 直接计算从世界到相机的旋转矩阵，避免了多次不必要的求逆操作,面向前方的相机使用12旋转矩阵即可，底部相机加入旋转矩阵3。
        tf::Matrix3x3 rot_matrix_cam_to_world = rot_matrix1 * rot_matrix2; //* rot_matrix3;

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
        if (temp_array.data[3] == 1)
        {
            Position_before.at<double>(0, 0) = temp_array.data[0] + 0.002749479296725727;
            Position_before.at<double>(1, 0) = temp_array.data[1] - 0.05780676994925782;
            Position_before.at<double>(2, 0) = temp_array.data[2] + 0.07578245909207494; // 将其转换到imu飞控所在位置
            // cout << "Position: (" << Position_before.at<double>(0, 0) << ", " << Position_before.at<double>(1, 0) << ", " << Position_before.at<double>(2, 0) << ")" << endl;
            Position_after = R * Position_before;
            // cout << R << endl;
            cout << "Position: (" << Position_after.at<double>(0, 0) << ", " << Position_after.at<double>(1, 0) << ", " << Position_after.at<double>(2, 0) << ")" << endl;
            publishDetectionResult(false); // 提取的函数，用于发布检测结果
        }
        else
        {
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
