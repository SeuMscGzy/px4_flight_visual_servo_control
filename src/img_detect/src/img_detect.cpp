#include "img_detect.h"

using namespace std;
using namespace std::chrono;
template <typename T>
void flipImage(vpImage<T> &src, vpImage<T> &dst, int flipCode)
{
  dst.resize(src.getHeight(), src.getWidth());

  if (flipCode == 0)
  {
    // 垂直翻转
    for (unsigned int i = 0; i < src.getHeight(); ++i)
    {
      for (unsigned int j = 0; j < src.getWidth(); ++j)
      {
        dst[src.getHeight() - i - 1][j] = src[i][j];
      }
    }
  }
  else if (flipCode > 0)
  {
    // 水平翻转
    for (unsigned int i = 0; i < src.getHeight(); ++i)
    {
      for (unsigned int j = 0; j < src.getWidth(); ++j)
      {
        dst[i][src.getWidth() - j - 1] = src[i][j];
      }
    }
  }
  else
  {
    // 同时水平和垂直翻转
    for (unsigned int i = 0; i < src.getHeight(); ++i)
    {
      for (unsigned int j = 0; j < src.getWidth(); ++j)
      {
        dst[src.getHeight() - i - 1][src.getWidth() - j - 1] = src[i][j];
      }
    }
  }
  // cout << "1" << endl;
}
// 常量定义
constexpr double PROCESSING_LATENCY = 0.045; // 40ms处理延迟
const Eigen::Vector3d POS_OFFSET{0.00625, -0.08892, 0.06430};
constexpr double DEFAULT_FX = 615.1674805;
constexpr double DEFAULT_FY = 615.1675415;
constexpr double DEFAULT_CX = 312.1889954;
constexpr double DEFAULT_CY = 243.4373779;
ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
    : nh_(nh), stop_thread(false), processing(false),
      lost_target(true), desired_yaw(0)
{
  // 参数服务器配置
  double fx, fy, cx, cy;
  nh_.param("/camera/fx", fx, DEFAULT_FX);
  nh_.param("/camera/fy", fy, DEFAULT_FY);
  nh_.param("/camera/cx", cx, DEFAULT_CX);
  nh_.param("/camera/cy", cy, DEFAULT_CY);

  config.disable_stream(RS2_STREAM_DEPTH);
  config.disable_stream(RS2_STREAM_INFRARED);
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 60);
  g.open(config);
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
  tag_detector.setAprilTagQuadDecimate(1.0);
  tag_detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
  tag_detector.setAprilTagNbThreads(4);
  tag_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  tag_detector.setZAlignedWithCameraAxis(false);

  // 坐标系初始化
  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();
  R_i2c << 0.00400, -0.03121, 0.99950,
      -0.99996, 0.00749, 0.00423,
      -0.00762, -0.99948, -0.03118;
  R_w2c = R_i2c;

  // ROS组件初始化
  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1, true);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &ObjectDetector::odomCallback, this, ros::TransportHints().tcpNoDelay());
  image_pub = nh.advertise<sensor_msgs::Image>("/camera_rect/image_rect", 1);
  timer = nh_.createTimer(ros::Duration(0.05), &ObjectDetector::timerCallback, this);
  worker_thread = thread(&ObjectDetector::processImages, this);
}

ObjectDetector::~ObjectDetector()
{
  stop_thread.store(true, std::memory_order_release);
  cv.notify_all();

  // 停止相机流
  if (worker_thread.joinable())
  {
    worker_thread.join();
  }
}

void ObjectDetector::start()
{
  ros::spin();
  ros::waitForShutdown();
}

void ObjectDetector::timerCallback(const ros::TimerEvent &)
{
  std::lock_guard<std::mutex> lock(data_mutex);
  processing.store(true, std::memory_order_release);
  cv.notify_one();
}

void ObjectDetector::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  std::lock_guard<std::mutex> lock(data_mutex);
  R_w2c = q.toRotationMatrix() * R_i2c;
}

void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  const auto delay =
      ros::Duration(PROCESSING_LATENCY) - (ros::Time::now() - ts);
  //cout << "Processing delay: " << delay.toSec() << " seconds." << endl;
  if (delay > ros::Duration(0))
  {
    this_thread::sleep_for(
        milliseconds(static_cast<long>(delay.toSec() * 1000)));
  }
  publishDetectionResult(ts, is_fault);
  processing.store(false, std::memory_order_release);
}

void ObjectDetector::publishDetectionResult(ros::Time &ts, bool is_fault)
{
  std_msgs::Float64MultiArray msg;
  msg.data = {Position_after.x(), Position_after.y(), Position_after.z(), ts.toSec(), static_cast<double>(is_fault || lost_target), desired_yaw};
  point_pub_.publish(msg);
  if (is_fault)
  {
    ROS_WARN_STREAM("Fault detected, skipping processing.");
  }
  else
  {
    ROS_INFO_STREAM("Processing completed successfully.");
  }
}

void ObjectDetector::processImages()
{
  while (!stop_thread.load(std::memory_order_acquire))
  {
    Eigen::Matrix3d R_w2c_temp;
    {
      unique_lock<std::mutex> lock(data_mutex);
      // 等待 processing 标志或退出标志
      cv.wait(lock, [&]
              { return processing.load(std::memory_order_acquire) ||
                       stop_thread.load(std::memory_order_acquire); });
      if (stop_thread.load(std::memory_order_acquire))
        break;
      // 复制共享变量，然后释放锁
      R_w2c_temp = R_w2c;
    } // 锁在此处释放
    try
    {
      vpImage<unsigned char> I;
      g.acquire(I);
      ros::Time image_timestamp = ros::Time::now();
      vpImage<unsigned char> flippedImage;
      flipImage(I, flippedImage, -1); // 假设-1表示同时水平和垂直翻转
      I = flippedImage;
      cv::Mat imageMat;
      // 使用Visp的函数将vpImage转换为cv::Mat
      vpImageConvert::convert(I, imageMat);

      std_msgs::Header header;
      header.stamp = ros::Time::now();
      //  现在你可以使用cv_bridge将cv::Mat转换为sensor_msgs/Image了
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", imageMat).toImageMsg();
      image_pub.publish(msg);

      vpImageFilter::gaussianFilter(I, 3, 3);
      std::vector<vpHomogeneousMatrix> cMo_vec;
      tag_detector.detect(I, 0.0795, cam, cMo_vec);
      if (cMo_vec.size() != 1)
      {
        baseProcess(image_timestamp, true);
        ROS_INFO_STREAM("No tag detected or multiple tags detected.");
        continue;
      }
      Position_before[0] = cMo_vec[0][0][3];
      Position_before[1] = cMo_vec[0][1][3];
      Position_before[2] = cMo_vec[0][2][3]; // 位置坐标
      // 应用位置偏移
      Position_before += POS_OFFSET;
      // 转换到世界坐标系
      Position_after = R_w2c_temp * Position_before;
      cout << "Position_after: " << Position_after.transpose() << endl;
      // 更新状态
      desired_yaw = 0;
      // cout << "yaw: " << yaw << endl;
      lost_target = false;
      // 后续处理
      baseProcess(image_timestamp, false);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Processing error: " << e.what());
      baseProcess(ros::Time::now(), true);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  ObjectDetector detector(nh);
  detector.start();
  return 0;
}