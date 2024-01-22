#include <ros/ros.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "quaternion_to_matrix");
    ros::NodeHandle nh;

    tf::Quaternion q(-0.5032544499127932, 0.5253568155686048, -0.48303903911102514, 0.48724579184769173);
    tf::Matrix3x3 m(q);

    double r00 = m[0][0], r01 = m[0][1], r02 = m[0][2];
    double r10 = m[1][0], r11 = m[1][1], r12 = m[1][2];
    double r20 = m[2][0], r21 = m[2][1], r22 = m[2][2];

    ROS_INFO("Rotation matrix:");
    ROS_INFO("%f, %f, %f", r00, r01, r02);
    ROS_INFO("%f, %f, %f", r10, r11, r12);
    ROS_INFO("%f, %f, %f", r20, r21, r22);

    return 0;
}
