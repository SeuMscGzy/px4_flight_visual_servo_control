#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

class ImageFlipper
{
public:
    ImageFlipper()
    {
        // Initialize image transport
        image_transport::ImageTransport it(nh);
        // Subscribe to input video feed and publish output video feed
        image_sub = it.subscribe("/camera/color/image_raw", 1, &ImageFlipper::imageCb, this);
        image_pub = it.advertise("/flipped_image", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Flip the image
        cv::Mat flipped_image;
        cv::flip(cv_ptr->image, flipped_image, -1); // Flip both horizontally and vertically

        // Update image message
        cv_bridge::CvImage out_msg;
        out_msg.header = cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever encoding matches the OpenCV image type
        out_msg.image = flipped_image; // Your cv::Mat

        // Publish the flipped image
        image_pub.publish(out_msg.toImageMsg());
    }

private:
    ros::NodeHandle nh;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_flipper");
    ImageFlipper ic;
    ros::spin();
    return 0;
}
