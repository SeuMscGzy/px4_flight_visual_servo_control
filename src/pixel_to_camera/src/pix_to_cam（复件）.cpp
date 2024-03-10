#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
using namespace std;

class AprilTagProcessor
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber forward_cam_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  ros::Publisher point_pub_;
  ros::Publisher loss_target_pub;

  cv::Mat K1, K2, Position_before, Position_after, R;
  nav_msgs::Path path_;
  int count1_, count2_;
  double time_start;
  double tagSize;
  std_msgs::Float64MultiArray point_;
  apriltag_detection_info_t cam_info;
  apriltag_pose_t pose;
  std::unique_ptr<apriltag_family_t, void (*)(apriltag_family_t *)> tag_family;
  std::unique_ptr<apriltag_detector_t, void (*)(apriltag_detector_t *)> tag_detector;
  void computeAndPrintTagInfo(apriltag_detection_t *det, const cv::Mat &K, const cv::Mat &R)
  {
    double u = det->c[0];
    double v = det->c[1];
    cam_info.det = det;
    double err = estimate_tag_pose(&cam_info, &pose);
    Position_before.at<double>(0, 0) = pose.t->data[0];
    Position_before.at<double>(1, 0) = pose.t->data[1];
    Position_before.at<double>(2, 0) = pose.t->data[2];
    Position_after = R * Position_before;
    cout << "Position: (" << Position_after.at<double>(0, 0) << ", " << Position_after.at<double>(1, 0) << ", " << Position_after.at<double>(2, 0) << ")" << endl;
  }

public:
  AprilTagProcessor() : nh_("~"), count1_(0), count2_(0), time_start(0.0), tagSize(0.1228), tag_family(tag36h11_create(), tag36h11_destroy),
                        tag_detector(apriltag_detector_create(), apriltag_detector_destroy)
  {
    // 初始化
    K1 = cv::Mat::zeros(3, 3, CV_64F);
    Position_before = cv::Mat::zeros(3, 1, CV_64F);
    Position_after = cv::Mat::zeros(3, 1, CV_64F);
    R = cv::Mat::zeros(3, 3, CV_64F);

    // 设置相机参数
    // 这是前置相机的参数
    K1.at<double>(0, 0) = 644.25537;
    K1.at<double>(1, 1) = 643.36945;
    K1.at<double>(0, 2) = 642.71234;
    K1.at<double>(1, 2) = 359.95392;

    // 这是下置相机的参数
    /*K1.at<double>(0, 0) = 492.095404;
    K1.at<double>(1, 1) = 504.902065;
    K1.at<double>(0, 2) = 340.221950;
    K1.at<double>(1, 2) = 151.780080;*/

    cam_info.tagsize = tagSize;
    cam_info.fx = K1.at<double>(0, 0); // 获取相机的焦距和像素
    cam_info.fy = K1.at<double>(1, 1);
    cam_info.cx = K1.at<double>(0, 2);
    cam_info.cy = K1.at<double>(1, 2);

    forward_cam_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &AprilTagProcessor::forwardimageCallback, this);
    // forward_cam_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &AprilTagProcessor::forwardimageCallback, this);
    odom_sub_ = nh_.subscribe("/vins_fusion/imu_propagate", 1, &AprilTagProcessor::odomCallback, this);
    // path_pub_ = nh_.advertise<nav_msgs::Path>("/apriltag_path", 10);
    point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_unfixed_delay", 10);
    apriltag_detector_add_family(tag_detector.get(), tag_family.get());
  }

  void forwardimageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      time_start = msg->header.stamp.now().toSec();

      // 将图像转换为灰度
      cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

      // 创建一个 image_u8_t 结构来包装 cv::Mat 的数据
      image_u8_t apriltag_image = {
          .width = frame.cols,
          .height = frame.rows,
          .stride = frame.step,
          .buf = frame.data};

      // 检测AprilTag
      zarray_t *detections_zarray = apriltag_detector_detect(tag_detector.get(), &apriltag_image);

      int numDetections = zarray_size(detections_zarray);
      if (numDetections == 1)
      {
        apriltag_detection_t *det1 = nullptr;
        zarray_get(detections_zarray, 0, &det1);
        computeAndPrintTagInfo(det1, K1, R);
        count1_++;
        publishDetectionResult(false); // 提取的函数，用于发布检测结果
      }
      else
      {
        count2_++;
        publishDetectionResult(true); // 提取的函数，用于发布检测结果
      }
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  void publishDetectionResult(bool lost)
  {
    point_.data.push_back(Position_after.at<double>(0, 0));
    point_.data.push_back(Position_after.at<double>(1, 0));
    point_.data.push_back(Position_after.at<double>(2, 0));
    point_.data.push_back(time_start);
    if (lost)
    {
      point_.data.push_back(1); // 1代表丢失目标时的值
    }
    else
    {
      point_.data.push_back(0); // 0代表精确的捕捉apriltag的值
    }
    point_pub_.publish(point_);
    point_.data.clear();
  }

  ~AprilTagProcessor()
  {
    apriltag_detector_destroy(tag_detector.get());
    tag36h11_destroy(tag_family.get());
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    tf::Quaternion q1(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 rot_matrix1(q1);
    tf::Matrix3x3 rot_matrix2(-0.0125414733481959, -0.0394725759038369, 0.999141945470099,
                              -0.999710607454322, -0.0200189744533413, -0.0133394904381207, 0.0205283411304649, -0.999020098102813, -0.0392100854000468);
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

  void spin()
  {
    ros::Rate rate(25);
    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  AprilTagProcessor processor;
  processor.spin();
  return 0;
}
