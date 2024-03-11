#include "controller.h"

using namespace std;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
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
  des_acc = des.a + Kp.asDiagonal() * (des.p - odom.p) + Kv.asDiagonal() * (des.v - odom.v); 
  u.acc_world = des_acc;
  u.q = imu.q * odom.q.inverse();
  u.des_yaw = fromQuaternion2yaw(u.q);
  return debug_msg_;
}
