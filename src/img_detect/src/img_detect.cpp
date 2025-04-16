#include "img_detect.h"

using namespace std;
using namespace std::chrono;

// 常量定义
constexpr double PROCESSING_LATENCY = 0.040; // 40ms处理延迟
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

  cfg.disable_stream(RS2_STREAM_DEPTH);
  cfg.disable_stream(RS2_STREAM_INFRARED);

  // 尝试启用 8 位灰度图像（Y8 格式）
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
  pipe.start(cfg);

  cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
  tag_sizes = {{0, 0.0795}};
  // AprilTag配置优化
  tf = tag36h11_create();
  td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;
  td->nthreads = std::min(2, static_cast<int>(std::thread::hardware_concurrency()));

  // 坐标系初始化
  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();
  R_c2a = Eigen::Matrix3d::Identity();
  R_i2c << 0.00400, -0.03121, 0.99950,
      -0.99996, 0.00749, 0.00423,
      -0.00762, -0.99948, -0.03118;
  R_w2c = R_i2c;

  for (const auto &kv : tag_sizes)
  {
    int id = kv.first;
    double tag_size = kv.second;
    cached_obj_pts_[id] = {
        cv::Point3d(-tag_size / 2, tag_size / 2, 0),
        cv::Point3d(tag_size / 2, tag_size / 2, 0),
        cv::Point3d(tag_size / 2, -tag_size / 2, 0),
        cv::Point3d(-tag_size / 2, -tag_size / 2, 0)};
  }

  // ROS组件初始化
  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1, true);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &ObjectDetector::odomCallback, this, ros::TransportHints().tcpNoDelay());
  image_pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);
  timer = nh_.createTimer(ros::Duration(0.05), &ObjectDetector::timerCallback, this);
  worker_thread = thread(&ObjectDetector::processImages, this);
}

ObjectDetector::~ObjectDetector()
{
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  stop_thread.store(true, std::memory_order_release);
  cv.notify_all();
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

void ObjectDetector::processSingleTag(apriltag_detection_t *det, Eigen::Matrix3d &R_c2a, Eigen::Vector3d &position)
{
  // 从缓存中查找预计算的物体点
  auto it = cached_obj_pts_.find(det->id);
  if (it == cached_obj_pts_.end())
  {
    throw std::runtime_error("No cached object points for tag id");
  }
  const std::vector<cv::Point3d> &obj_pts = it->second;

  vector<cv::Point2d> img_pts = {
      {det->p[0][0], det->p[0][1]},
      {det->p[1][0], det->p[1][1]},
      {det->p[2][0], det->p[2][1]},
      {det->p[3][0], det->p[3][1]}};

  if (!cv::solvePnP(obj_pts, img_pts, cameraMatrix, distCoeffs, rvec_, tvec_,
                    false, cv::SOLVEPNP_IPPE_SQUARE))
  {
    throw std::runtime_error("solvePnP failed");
  }

  // 这里尝试原地转换：直接用 rvec 存储旋转矩阵
  cv::Rodrigues(rvec_, rvec_); // rvec 现在存储 3x3 的旋转矩阵
  cv::cv2eigen(rvec_, R_c2a);  // 直接转换到 Eigen
  position = Eigen::Vector3d(tvec_.at<double>(0), tvec_.at<double>(1),
                             tvec_.at<double>(2));
}

void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  const auto delay =
      ros::Duration(PROCESSING_LATENCY) - (ros::Time::now() - ts);
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
  cv::Mat distorted_image;
  while (!stop_thread.load(std::memory_order_acquire))
  {
    distorted_image.release(); // 显式释放
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
      rs2::frameset frames = pipe.wait_for_frames();
      rs2::video_frame color_frame = frames.get_color_frame();
      ros::Time image_timestamp = ros::Time::now();
      int width = color_frame.get_width();
      int height = color_frame.get_height();
      const void *data = color_frame.get_data();

      // 使用 RGB 格式构造 OpenCV 图像（3通道）
      cv::Mat colorImage(cv::Size(width, height), CV_8UC3, (void *)data, cv::Mat::AUTO_STEP);

      // 转换为灰度图像，使用 cv::COLOR_RGB2GRAY
      cv::Mat distorted_image;
      cv::cvtColor(colorImage, distorted_image, cv::COLOR_RGB2GRAY);
      // 获取图像
      if (distorted_image.empty())
      {
        baseProcess(image_timestamp, true);
        continue;
      }
      // 垂直翻转图像
      cv::flip(distorted_image, distorted_image, 0);
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      // 现在你可以使用cv_bridge将cv::Mat转换为sensor_msgs/Image了
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", distorted_image).toImageMsg();
      image_pub.publish(msg);
      // AprilTag检测
      image_u8_t apriltag_image = {.width = distorted_image.cols,
                                   .height = distorted_image.rows,
                                   .stride = static_cast<int>(distorted_image.step),
                                   .buf = distorted_image.data};
      zarray_t *raw_detections =
          apriltag_detector_detect(td, &apriltag_image);
      unique_ptr<zarray_t, decltype(&apriltag_detections_destroy)> detections(
          raw_detections, apriltag_detections_destroy);
      // 处理检测结果
      const int detection_count = zarray_size(detections.get());
      ROS_DEBUG_STREAM("Detected tags: " << detection_count);
      if (detection_count != 1)
      {
        baseProcess(image_timestamp, true);
        continue;
      }
      apriltag_detection_t *det;
      zarray_get(detections.get(), 0, &det);
      // 可选：如果你仍然需要确保检测到的标签在预期范围内，可以检查 tag_sizes 是否包含此 id
      if (tag_sizes.find(det->id) == tag_sizes.end())
      {
        baseProcess(image_timestamp, true);
        continue;
      }
      processSingleTag(det, R_c2a, Position_before);
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