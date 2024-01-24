#include "controller.h"

using namespace std;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(const Desired_State_t &des,
                                                             const Odom_Data_t &odom,
                                                             const Imu_Data_t &imu,
                                                             Controller_Output_t &u, bool is_cmd_mode_)
{
  /* WRITE YOUR CODE HERE */
  // compute disired acceleration
  Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
  Eigen::Vector3d Kp, Kv;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
  des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p); // 这里需要修改
  des_acc += Eigen::Vector3d(0, 0, param_.gra);
  // cout << des.p << endl;
  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
  if (des.p[2] < -0.2 && odom.p[2] < 0.1 && des_acc[2] < param_.gra && is_cmd_mode_==false) // 在地上或快降落到地上且推杆在底部或中部且为auto_hover模式
  {
    u.thrust = 0.15;
  }
  double roll, pitch, yaw, yaw_imu;
  double yaw_odom = fromQuaternion2yaw(odom.q);
  double sin = std::sin(yaw_odom);
  double cos = std::cos(yaw_odom);
  roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra; // 线性化后的情况
  pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
  // yaw_imu = fromQuaternion2yaw(imu.q);
  cout << "pitch:" << pitch << " roll:" << roll << " yaw_odom:" << yaw_odom << " yaw_imu:" << yaw_imu << endl;
  //  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
  //    * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
  //    * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
  Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  u.q = imu.q * odom.q.inverse() * q; //??????
  // double yaw_desired = fromQuaternion2yaw(u.q);
  // cout << "odom.q.inverse_yaw:" << fromQuaternion2yaw(odom.q.inverse()) << " des_yaw:" << des.yaw << " yaw_desired:" << yaw_desired << endl;
  /* WRITE YOUR CODE HERE */

  // used for debug
  //  debug_msg_.des_p_x = des.p(0);
  //  debug_msg_.des_p_y = des.p(1);
  //  debug_msg_.des_p_z = des.p(2);

  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);

  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  // cout << debug_msg_.des_q_x << " " << debug_msg_.des_q_y << " " << debug_msg_.des_q_z << " " << debug_msg_.des_q_w << " " << endl;

  debug_msg_.des_thr = u.thrust;
  // cout << u.thrust << endl;
  //  Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    if (thr > 0.15)
    {
      /***********************************/
      /* Model: est_a(2) = thr1acc_ * thr */
      /***********************************/
      double gamma = 1 / (rho2_ + thr * P_ * thr);
      double K = gamma * P_ * thr;
      thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
      P_ = (1 - K * thr) * P_ / rho2_;
    }
    // cout << thr2acc_ << endl;
    return true;
  }
  return false;
}

void LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}
