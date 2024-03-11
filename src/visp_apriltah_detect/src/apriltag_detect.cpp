#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImageConvert.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
using namespace std;
double data1[] = {0.011763502382341945f, -0.03427874295798661f, 0.999343078123285f,
                  -0.999463220176014f, 0.030157061398510737f, 0.01279934229561297f,
                  -0.030575995929723154f, -0.9989572160152946f, -0.03390559015676936f};
class AprilTagDetector
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    vpDisplayX *d = NULL;
    vpImage<unsigned char> I;
    std_msgs::Float64MultiArray point_;
    ros::Subscriber odom_sub_;
    ros::Publisher point_pub_;
    cv::Mat Position_before = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Position_after = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    vpCameraParameters cam_params;
    double tagSize = 0.0885; // AprilTag的尺寸（单位：米）
    double alpha;
    vpDetectorAprilTag detector{vpDetectorAprilTag::TAG_36h11};
    std::vector<vpHomogeneousMatrix> cMo;

public:
    AprilTagDetector() : it_(nh_)
    {
        cv::Mat tempMat(3, 3, CV_64F, data1);
        R = tempMat;
        image_sub_ = it_.subscribe("/flipped_image", 1, &AprilTagDetector::imageCb, this);
        odom_sub_ = nh_.subscribe("/vins_fusion/imu_propagate", 1, &AprilTagDetector::odomCallback, this);
        point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_unfixed_delay", 10);
        // 直接使用给定的内参数值初始化相机参数
        alpha = 1; // 缩放比例

        // 假设原始相机内参是这样的
        double fx = 625.5179846719374, fy = 619.7330458544326, cx = 637.3374051805021, cy = 363.5448204145972;

        // 调整内参以匹配新的图像尺寸
        double fx_new = fx * alpha;
        double fy_new = fy * alpha;
        double cx_new = cx * alpha;
        double cy_new = cy * alpha;

        // 使用新的内参初始化相机参数
        cam_params.initPersProjWithoutDistortion(fx_new, fy_new, cx_new, cy_new);
    }

    ~AprilTagDetector()
    {
        if (d)
        {
            delete d;
        }
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

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // ros::Time start_time = ros::Time::now(); // 开始时间
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

            // 调整图像大小
            cv::Mat resized_image;
            cv::resize(cv_ptr->image, resized_image, cv::Size(), alpha, alpha); // 将图像缩小为原来的50%

            // 将cv::Mat转换为vpImage
            vpImage<unsigned char> I;
            vpImageConvert::convert(resized_image, I);

            // ros::Time end_time = ros::Time::now(); // 结束时间

            // 计算运行时间（以秒为单位）
            // ros::Duration duration = end_time - start_time;
            /*if (d == NULL)
            {
                d = new vpDisplayX(I);
                vpDisplay::setTitle(I, "AprilTag detection");
            }*/
            // vpDisplay::display(I);
            // 调用detect方法，额外传入相机参数和标签尺寸
            // ros::Time start_time = ros::Time::now(); // 开始时间

            cMo.clear();
            // 调用检测函数
            detector.detect(I, tagSize, cam_params, cMo);

            // ros::Time end_time = ros::Time::now(); // 结束时间

            // 计算运行时间（以秒为单位）
            // ros::Duration duration = end_time - start_time;

            // ROS_INFO("Detection took %f seconds.", duration.toSec());

            if (!cMo.empty())
            {
                // 直接打印检测到的单个AprilTag的相机坐标
                // std::cout << "AprilTag pose (camera frame):" << std::endl;
                // cMo[0].print();
                vpTranslationVector t;
                cMo[0].extract(t);
                Position_before.at<double>(0, 0) = t[0] + 0.002749479296725727;
                Position_before.at<double>(1, 0) = t[1] - 0.05780676994925782;
                Position_before.at<double>(2, 0) = t[2] + 0.07578245909207494; // 将其转换到imu飞控所在位置
                // cout << "Position: (" << Position_before.at<double>(0, 0) << ", " << Position_before.at<double>(1, 0) << ", " << Position_before.at<double>(2, 0) << ")" << endl;
                Position_after = R * Position_before;
                //cout << R << endl;
                //cout << "Position: (" << Position_after.at<double>(0, 0) << ", " << Position_after.at<double>(1, 0) << ", " << Position_after.at<double>(2, 0) << ")" << endl;
                publishDetectionResult(false); // 提取的函数，用于发布检测结果
            }
            else
            {
                publishDetectionResult(true); // 提取的函数，用于发布检测结果
            }
            // vpDisplay::flush(I);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
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
