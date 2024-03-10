#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "setpoint_accel_yaw_node");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到/mavros/setpoint_raw/local话题，队列大小为10
    ros::Publisher pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 确保有订阅者连接
    while (pub.getNumSubscribers() == 0) {
        ROS_INFO("Waiting for subscribers...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    // 初始化PositionTarget消息
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    // 设置你想要的加速度值
    msg.acceleration_or_force.x = 1.0; // X轴加速度
    msg.acceleration_or_force.y = 1.0; // Y轴加速度
    msg.acceleration_or_force.z = 1.0; // Z轴加速度
    // 设置偏航角（以弧度为单位）
    msg.yaw = 1.57; // 偏航角，例如，1.57弧度约等于90度

    // 发布消息
    pub.publish(msg);

    ROS_INFO("Published setpoint message with acceleration and yaw.");
    ros::spinOnce();

    return 0;
}
