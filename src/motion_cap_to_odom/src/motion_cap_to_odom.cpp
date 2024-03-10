#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher odom_pub;

void callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg,
              const geometry_msgs::TwistStamped::ConstPtr &twist_msg)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = pose_msg->header.stamp; // 可以选择pose或twist的时间戳
    odom.header.frame_id = "world";             // 根据需要修改
    odom.child_frame_id = "";                   // 或您的机器人的相应帧ID

    // 设置位姿
    odom.pose.pose = pose_msg->pose;

    // 设置速度
    odom.twist.twist = twist_msg->twist;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_pose_and_velocity");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 50);

    // 设置message_filters订阅器
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "/vrpn_client_node/MCServer/5/pose", 10);
    message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub(nh, "/vrpn_client_node/MCServer/5/velocity", 10);

    // 使用ApproximateTime策略同步消息
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(15), pose_sub, twist_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
