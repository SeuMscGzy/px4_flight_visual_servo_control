#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;
class DelayFixer
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    std_msgs::Float64MultiArray vec;
    ros::Timer timer;

public:
    DelayFixer()
    {
        sub = nh.subscribe("/point_with_unfixed_delay", 1, &DelayFixer::Callback, this);
        pub = nh.advertise<std_msgs::Float64MultiArray>("/delay_sent_50ms", 1);
        // timer = nh.createTimer(ros::Duration(0.05), &DelayFixer::point_pub, this);
    }

    void Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        vec.data.clear();
        vec.data = msg->data;
        pub.publish(vec);
        // cout << "pub spent: " << 1000 * (ros::Time::now().toSec() - vec.data[3]) << " ms" << endl;
    }

    void point_pub(const ros::TimerEvent &)
    {
        // cout << "Time spent: " << 1000 * (ros::Time::now().toSec() - vec.data[3]) << " ms" << endl;
        // pub.publish(vec);
    }

    void spin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delay_fixed");
    DelayFixer delay_fixer;
    delay_fixer.spin();
    return 0;
}
