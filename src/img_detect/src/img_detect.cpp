#include <visp3/core/vpConfig.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <std_msgs/Float64MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visp3/core/vpImageTools.h>
// #undef VISP_HAVE_V4L2
// #undef VISP_HAVE_DC1394
// #undef VISP_HAVE_CMU1394
// #undef VISP_HAVE_FLYCAPTURE
// #undef VISP_HAVE_REALSENSE2
// #undef VISP_HAVE_OPENCV
using namespace std;
template <typename T>
void flipImage(vpImage<T> &src, vpImage<T> &dst, int flipCode)
{
    dst.resize(src.getHeight(), src.getWidth());

    if (flipCode == 0)
    {
        // 垂直翻转
        for (unsigned int i = 0; i < src.getHeight(); ++i)
        {
            for (unsigned int j = 0; j < src.getWidth(); ++j)
            {
                dst[src.getHeight() - i - 1][j] = src[i][j];
            }
        }
    }
    else if (flipCode > 0)
    {
        // 水平翻转
        for (unsigned int i = 0; i < src.getHeight(); ++i)
        {
            for (unsigned int j = 0; j < src.getWidth(); ++j)
            {
                dst[i][src.getWidth() - j - 1] = src[i][j];
            }
        }
    }
    else
    {
        // 同时水平和垂直翻转
        for (unsigned int i = 0; i < src.getHeight(); ++i)
        {
            for (unsigned int j = 0; j < src.getWidth(); ++j)
            {
                dst[src.getHeight() - i - 1][src.getWidth() - j - 1] = src[i][j];
            }
        }
    }
    // cout << "1" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    ros::Publisher pbvs_publisher = nh.advertise<std_msgs::Float64MultiArray>("/object_pose", 1);
    ros::Publisher ibvs_publisher = nh.advertise<std_msgs::Float64MultiArray>("/object_pixel", 1);
    std_msgs::Float64MultiArray ros_pbvs_msg;
    std_msgs::Float64MultiArray ros_ibvs_msg;
    ros_pbvs_msg.data.resize(6, 0.0);
    ros_ibvs_msg.data.resize(8, 0.0);
#if defined(VISP_HAVE_APRILTAG) &&                                                                                                   \
    (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || (VISP_HAVE_OPENCV_VERSION >= 0x020100) || \
     defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2))
    int opt_device = 0; // For OpenCV and V4l2 grabber to set the camera device
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    double tagSize = 0.075;
    float quad_decimate = 1.0;
    int nThreads = 1;
    std::string intrinsic_file = "";
    std::string camera_name = "";
    bool display_tag = false;
    int color_id = -1;
    unsigned int thickness = 2;
    bool align_frame = false;
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
    bool display_off = true;
    std::cout << "Warning: There is no 3rd party (X11, GDI or openCV) to dislay images..." << std::endl;
#else
    bool display_off = false;
#endif
    vpImage<unsigned char> I;
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
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
            std::cout << " [--display_off] [--color <color id>] [--thickness <line thickness>]";
#endif
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
#if defined(VISP_HAVE_REALSENSE2)
        (void)opt_device; // To avoid non used warning
        std::cout << "Use Realsense 2 grabber" << std::endl;
        vpRealSense2 g;
        rs2::config config;
        config.disable_stream(RS2_STREAM_DEPTH);
        config.disable_stream(RS2_STREAM_INFRARED);
        config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
        g.open(config);
        g.acquire(I);
        vpImage<unsigned char> flippedImage;
        flipImage(I, flippedImage, -1); // 假设-1表示同时水平和垂直翻转
        // cout << "here" << endl;
        //  如果需要，可以在这里将翻转后的图像赋值回I
        I = flippedImage;
        std::cout << "Read camera parameters from Realsense device" << std::endl;
        cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
#elif defined(VISP_HAVE_OPENCV)
        std::cout << "Use OpenCV grabber on device " << opt_device << std::endl;
        cv::VideoCapture g(opt_device); // Open the default camera
        if (!g.isOpened())
        { // Check if we succeeded
            std::cout << "Failed to open the camera" << std::endl;
            return -1;
        }
        cv::Mat frame;
        g >> frame; // get a new frame from camera
        vpImageConvert::convert(frame, I);
#endif
        std::cout << cam << std::endl;
        std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
        std::cout << "tagFamily: " << tagFamily << std::endl;
        std::cout << "nThreads : " << nThreads << std::endl;
        std::cout << "Z aligned: " << align_frame << std::endl;
        vpDisplay *d = NULL;
        if (!display_off)
        {
#ifdef VISP_HAVE_X11
            d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
            d = new vpDisplayGDI(I);
#elif defined(VISP_HAVE_OPENCV)
            d = new vpDisplayOpenCV(I);
#endif
        }
        vpDetectorAprilTag detector(tagFamily);
        detector.setAprilTagQuadDecimate(quad_decimate);
        detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
        detector.setAprilTagNbThreads(nThreads);
        detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
        detector.setZAlignedWithCameraAxis(align_frame);
        std::vector<double> time_vec;
        for (;;)
        {
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
            g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
            g >> frame;
            vpImageConvert::convert(frame, I);
#endif
            vpImage<unsigned char> flippedImage;
            flipImage(I, flippedImage, -1); // 假设-1表示同时水平和垂直翻转
            // cout << "here" << endl;
            // 如果需要，可以在这里将翻转后的图像赋值回I
            I = flippedImage;
            vpDisplay::display(I);
            double t = vpTime::measureTimeMs();
            std::vector<vpHomogeneousMatrix> cMo_vec;
            std::vector<vpImagePoint> pixel_vec;
            vpHomogeneousMatrix pose_matrix;
            detector.detect(I, tagSize, cam, cMo_vec);

            t = vpTime::measureTimeMs() - t;
            time_vec.push_back(t);
            if (cMo_vec.size() != 0)
            {
                pixel_vec = detector.getPolygon(0);
                pose_matrix = cMo_vec[0];
                vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);
                for (size_t i = 0; i < pixel_vec.size(); i++)
                {
                    vpDisplay::displayCross(I, pixel_vec[i], 14, vpColor::red, 3);
                    std::ostringstream number;
                    number << i + 1;
                    vpDisplay::displayText(I, pixel_vec[i] + vpImagePoint(15, 5), number.str(), vpColor::blue);
                }
                ros_ibvs_msg.data[0] = pixel_vec[0].get_u();
                ros_ibvs_msg.data[1] = pixel_vec[0].get_v();
                ros_ibvs_msg.data[2] = pixel_vec[1].get_u();
                ros_ibvs_msg.data[3] = pixel_vec[1].get_v();
                ros_ibvs_msg.data[4] = pixel_vec[2].get_u();
                ros_ibvs_msg.data[5] = pixel_vec[2].get_v();
                ros_ibvs_msg.data[6] = pixel_vec[3].get_u();
                ros_ibvs_msg.data[7] = pixel_vec[3].get_v();
                ros_pbvs_msg.data[0] = pose_matrix[0][3];
                ros_pbvs_msg.data[1] = pose_matrix[1][3];
                ros_pbvs_msg.data[2] = pose_matrix[2][3];
                ros_pbvs_msg.data[3] = 1;
            }
            else
            {
                ros_pbvs_msg.data[3] = 0;
            }
            vpDisplay::flush(I);
            if (vpDisplay::getClick(I, false))
                break;
            pbvs_publisher.publish(ros_pbvs_msg);
            ibvs_publisher.publish(ros_ibvs_msg);
        }
        std::cout << "Benchmark computation time" << std::endl;
        std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
                  << " ; " << vpMath::getMedian(time_vec) << " ms"
                  << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;
        if (!display_off)
            delete d;
    }
    catch (const vpException &e)
    {
        std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
    return EXIT_SUCCESS;
#else
    (void)argc;
    (void)argv;
#ifndef VISP_HAVE_APRILTAG
    std::cout << "Enable Apriltag support, configure and build ViSP to run this tutorial" << std::endl;
#else
    std::cout << "Install a 3rd party dedicated to frame grabbing (dc1394, cmu1394, v4l2, OpenCV, FlyCapture, Realsense2), configure and build ViSP again to use this example" << std::endl;
#endif
#endif
    return EXIT_SUCCESS;
}