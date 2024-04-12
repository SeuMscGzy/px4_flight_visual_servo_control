#include <visp3/core/vpConfig.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <std_msgs/Float64MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visp3/core/vpImageTools.h>
using namespace std;
vpImage<unsigned char> I;
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        vpImageConvert::convert(cv_ptr->image, I);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Publisher pbvs_publisher = nh.advertise<std_msgs::Float64MultiArray>("/object_pose", 1);
    ros::Subscriber imgsub = nh.subscribe("/image", 1, imageCallback);
    std_msgs::Float64MultiArray ros_pbvs_msg;
    ros_pbvs_msg.data.resize(6, 0.0);
    int opt_device = 0; // For OpenCV and V4l2 grabber to set the camera device
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    double tagSize = 0.111;
    float quad_decimate = 1.0;
    int nThreads = 2;
    std::string intrinsic_file = "";
    std::string camera_name = "";
    bool display_tag = false;
    int color_id = -1;
    unsigned int thickness = 2;
    bool align_frame = false;
    bool display_off = false;
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--pose_method" && i + 1 < argc)
        {
            poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc)
        {
            tagSize = atof(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--camera_device" && i + 1 < argc)
        {
            opt_device = atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc)
        {
            quad_decimate = (float)atof(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc)
        {
            nThreads = atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc)
        {
            intrinsic_file = std::string(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc)
        {
            camera_name = std::string(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--display_tag")
        {
            display_tag = true;
        }
        else if (std::string(argv[i]) == "--display_off")
        {
            display_off = true;
        }
        else if (std::string(argv[i]) == "--color" && i + 1 < argc)
        {
            color_id = atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--thickness" && i + 1 < argc)
        {
            thickness = (unsigned int)atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc)
        {
            tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--z_aligned")
        {
            align_frame = true;
        }
        else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h")
        {
            std::cout << "Usage: " << argv[0]
                      << " [--camera_device <camera device> (default: 0)]"
                      << " [--tag_size <tag_size in m> (default: 0.053)]"
                         " [--quad_decimate <quad_decimate> (default: 1)]"
                         " [--nthreads <nb> (default: 1)]"
                         " [--intrinsic <intrinsic file> (default: empty)]"
                         " [--camera_name <camera name>  (default: empty)]"
                         " [--pose_method <method> (0: HOMOGRAPHY, 1: HOMOGRAPHY_VIRTUAL_VS, "
                         " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS, "
                         " 4: BEST_RESIDUAL_VIRTUAL_VS, 5: HOMOGRAPHY_ORTHOGONAL_ITERATION) (default: 0)]"
                         " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10 (DEPRECATED), 2: TAG_36ARTOOLKIT (DEPRECATED),"
                         " 3: TAG_25h9, 4: TAG_25h7 (DEPRECATED), 5: TAG_16h5, 6: TAG_CIRCLE21h7, 7: TAG_CIRCLE49h12,"
                         " 8: TAG_CUSTOM48h12, 9: TAG_STANDARD41h12, 10: TAG_STANDARD52h13) (default: 0)]"
                         " [--display_tag] [--z_aligned]";
            std::cout << " [--display_off] [--color <color id>] [--thickness <line thickness>]";
            std::cout << " [--help]" << std::endl;
            return EXIT_SUCCESS;
        }
    }
    try
    {
        vpCameraParameters cam;
        cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
        vpXmlParserCamera parser;
        if (!intrinsic_file.empty() && !camera_name.empty())
            parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
        std::cout << cam << std::endl;
        std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
        std::cout << "tagFamily: " << tagFamily << std::endl;
        std::cout << "nThreads : " << nThreads << std::endl;
        std::cout << "Z aligned: " << align_frame << std::endl;
        vpDisplay *d = NULL;
        if (!display_off)
        {
            d = new vpDisplayX(I);
        }
        vpDetectorAprilTag detector(tagFamily);
        detector.setAprilTagQuadDecimate(quad_decimate);
        detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
        detector.setAprilTagNbThreads(nThreads);
        detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
        detector.setZAlignedWithCameraAxis(align_frame);
        ros::Rate rate2(20);
        while (ros::ok())
        {
            double img_recieve = ros::Time::now().toSec();
            double image_time = ros::Time::now().toSec();
            // vpDisplay::display(I);
            std::vector<vpHomogeneousMatrix> cMo_vec;
            vpHomogeneousMatrix pose_matrix;
            detector.detect(I, tagSize, cam, cMo_vec);
            /*double img_over = ros::Time::now().toSec();
            cout << "over-recieve: " << 1000 * (img_over - img_recieve) << " ms" << endl;*/
            if (cMo_vec.size() != 0)
            {
                pose_matrix = cMo_vec[0];
                ros_pbvs_msg.data[0] = pose_matrix[0][3];
                ros_pbvs_msg.data[1] = pose_matrix[1][3];
                ros_pbvs_msg.data[2] = pose_matrix[2][3];
                ros_pbvs_msg.data[3] = 1;
                ros_pbvs_msg.data[4] = image_time;
            }
            else
            {
                ros_pbvs_msg.data[3] = 0;
                ros_pbvs_msg.data[4] = image_time;
            }
            // vpDisplay::flush(I);
            if (vpDisplay::getClick(I, false))
                break;
            // double img_before_pub = ros::Time::now().toSec();
            if (24000 - 1000000 * (ros::Time::now().toSec() - img_recieve) > 0)
            {
                usleep(double(25000) - 1000000 * (ros::Time::now().toSec() - img_recieve));
            }
            pbvs_publisher.publish(ros_pbvs_msg);
            rate2.sleep();
            /*double img_send = ros::Time::now().toSec();
            cout << "send-recieve: " << 1000 * (img_send - img_recieve) << " ms" << endl;*/
        }
        if (!display_off)
            delete d;
    }
    catch (const vpException &e)
    {
        std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
    return EXIT_SUCCESS;
}