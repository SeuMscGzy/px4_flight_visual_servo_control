#include "controller.h"
#include <Eigen/Core>
using namespace std;

int pos_vel_control_counter = 0; // Position-velocity controller's counter.
const double dt = 0.02;          // Time interval of the velocity control loop: 50 Hz
double desire_v_x = 0;
double desire_v_y = 0;
double desire_v_z = 0;
const double position_controller_gain_horizontal = 0.9; // Horizontal position proportional control gain
const double position_controller_gain_vertical = 1;     // Vertical position proportional control gain
Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
Eigen::Vector3d last_des_acc(0.0, 0.0, 0.0);
Eigen::Vector3d des_vel(0.0, 0.0, 0.0);
Eigen::Vector3d Kp, Kv;
double last_pos_vel_time = 0;

class PIDController
{
public:
  double Kp, Ki, Kd;
  double integral_error;

  PIDController(double Kp_, double Ki_, double Kd_) : integral_error(0)
  {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
  }

  double update(double error)
  {
    integral_error += error * dt;
    if (abs(integral_error) > 1)
    {
      integral_error = 1 * integral_error / abs(integral_error);
    }
    double u_pid = Kp * error + Ki * integral_error;
    if (abs(u_pid > 2))
    {
      u_pid = 2 * u_pid / abs(u_pid);
    }
    return u_pid;
  }
  void init()
  {
    integral_error = 0;
  }
};
PIDController velocity_controller_x(2, 0.4, 0);  
PIDController velocity_controller_y(2, 0.4, 0); 
PIDController velocity_controller_z(4, 2, 0); 

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
                                                             Controller_Output_t &u, int state_count, bool in_landing_)
{
  if (state_count == 1)
  {
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    u.acc[0] = 0;
    u.acc[1] = 0;
    u.acc[2] = -1;
  }
  else if (state_count == 2)
  {
    if (pos_vel_control_counter % 2 == 0)
    {
      // cout << "pos_time: " << 1000 * (ros::Time::now().toSec() - last_pos_vel_time) << endl;
      // last_pos_vel_time = ros::Time::now().toSec();
      desire_v_x = position_controller_gain_horizontal * (des.p[0] - odom.p[0]);
      desire_v_y = position_controller_gain_horizontal * (des.p[1] - odom.p[1]);
      desire_v_z = position_controller_gain_vertical * (des.p[2] - odom.p[2]);
      pos_vel_control_counter = 0;
      des_acc[0] = velocity_controller_x.update(desire_v_x - odom.v[0]);
      des_acc[1] = velocity_controller_y.update(desire_v_y - odom.v[1]);
      des_acc[2] = velocity_controller_z.update(desire_v_z - odom.v[2]);
      last_des_acc = des_acc;
    }
    else
    {
      des_acc = last_des_acc;
    }
  }
  else
  {
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    des_acc = des.a;
  }
  // cout << des_acc[2] << " " << pos_vel_control_counter << endl;
  pos_vel_control_counter++;
  des_acc += Eigen::Vector3d(0, 0, param_.gra);
  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
  /*Eigen::Quaterniond q_w_b(odom.q);
  Eigen::Matrix3d R_w_b = q_w_b.toRotationMatrix();
  Eigen::Vector3d body_z_real;

  body_z_real = R_w_b.col(2);
  body_z_real = body_z_real.normalized();
  double thrust = des_acc.dot(body_z_real); 
  Eigen::Vector3d thr;
  thr(2) = thrust;
  // u.thrust = computeDesiredCollectiveThrustSignal(thr);
  thrust = computeDesiredCollectiveThrustSignal(thr);
  Eigen::Vector3d body_z = des_acc.normalized();
  Eigen::Vector3d body_x_temp(std::cos(des.yaw), std::sin(des.yaw), 0);
  Eigen::Vector3d body_y = body_z.cross(body_x_temp);
  body_y = body_y.normalized();
  Eigen::Vector3d body_x = body_y.cross(body_z);
  body_x = body_x.normalized();
  Eigen::Matrix3d R_w_b_des;
  R_w_b_des << body_x, body_y, body_z;
  Eigen::Quaterniond q_des_se3(R_w_b_des);*/
  if (state_count == 1)
  {
    u.thrust = 0.01;
  }
  if ((last_state_count == 1 && state_count == 2) || in_landing_)
  {
    enter_count = 0;
  }
  if ((state_count != 3 && des.p[2] < -0.2 && odom.p[2] < 0.1 && des_acc[2] < param_.gra) || enter_count < 200) // 在地上或快降落到地上且推杆在底部或中部且为auto_hover模式
  {
    // cout << odom.p[2] << endl;
    // cout << des_acc[2] << endl;
    in_the_slow_thrust = true;
    enter_count++;
    if (enter_count > 200)
    {
      enter_count = 200;
    }
    last_thrust *= 0.995;
    if (last_thrust < 0.01)
    {
      last_thrust = 0.01;
    }
    u.thrust = last_thrust;
  }
  else
  {
    in_the_slow_thrust = false;
  }
  if (last_in_the_slow_thrust == true && in_the_slow_thrust == false && state_count != 3 && !in_landing_) // Gradually accelerate for takeoff
  {
    takeoff_count = 0;
  }
  if (takeoff_count < 500)
  {
    takeoff_count++;
    u.thrust = u.thrust * takeoff_count * 0.01 * 0.2; // Gradually ramp thrust from 1% to 100%
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
  }
  last_in_the_slow_thrust = in_the_slow_thrust;
  last_state_count = state_count;
  last_thrust = u.thrust;
  // SE(3) in hovering state
  double roll, pitch;
  double yaw_odom = fromQuaternion2yaw(odom.q);
  double sin = std::sin(yaw_odom);
  double cos = std::cos(yaw_odom);
  roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
  pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
  // yaw = fromQuaternion2yaw(des.q);
  Eigen::Quaterniond q_des = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  u.q = imu.q * odom.q.inverse() * q_des;
  /*
  // SE(3)
  Vector3d b3d = des_acc.normalized();
  Vector3d a_psi{cos(0), sin(0), 0}; //do not change yaw in the experiment cases

  Vector3d b2d = b3d.cross(a_psi).normalized();
  Vector3d b1d = b2d.cross(b3d);

  Matrix3d R_d;
  R_d.col(0) = b1d;
  R_d.col(1) = b2d;
  R_d.col(2) = b3d;
  Quaterniond q_SE3(R_d);
  u.q = imu.q * odom.q.inverse() * q_SE3;
  */
  // Eigen::Vector3d euler_angles = q_des.toRotationMatrix().eulerAngles(0, 1, 2);
  if (takeoff_count >= 500)
  {
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  }
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }

  // used for debug
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
  return debug_msg_;
}

double LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
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
    if (thr > 0.01 && takeoff_count >= 500)
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
