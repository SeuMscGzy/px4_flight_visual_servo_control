## Environment Setup

1. **Install ROS (Preferably ROS Noetic)**  
   Make sure you have ROS Noetic installed and properly configured on your system.

2. **Install MAVROS Development Environment**  
   Follow the [MAVROS installation guide](http://wiki.ros.org/mavros) to set up the MAVROS package, ensuring all required dependencies are met.

3. **Install ViSP Dependencies**  
   This project relies on ViSP libraries for certain computer vision functionalities. Please refer to the official [ViSP documentation](https://visp.inria.fr/) or use your OS package manager for installation.

4. **Install IIR Filter Dependencies**  
   Ensure you have installed the libraries required for [IIR filtering](https://github.com/berndporr/iir1). Check your package manager or the library’s documentation for installation details.

5. **Clone or Download This Repository**  
   ```bash
   cd <your_workspace>
   git clone https://github.com/SeuMscGzy/px4_flight_visual_servo_control.git
   catkin_make  # or colcon build, depending on your setup

## Introduction

1. **Visual Servo Method**
   
   ---RAID-AgiVS: In the RAID-AgiVS_with_d_term and RAID-AgiVS_without_d_term package.
   
   ---PID: In the pid package.
   
   ---AIC: In the aic_with_d package.

2. **Image Processing**
   
   ---Obtain the image cordinate of the Apriltag: In the img_detect package.
   
   ---Obtian the relative position error between the drone and Apriltag: In the Pos_err_cam2world package.

3. **Visualization**
   
   ---Get the video shot by the camera: In the video_creater package.
   
   ---Get tracking data: In the record_curves package.
   
   ---Get data from motion capture system: In the vrpn_client_ros package.

4. **Flight Logic**
   
   ---Flight status determination: In the px4ctrl package. Thanks for the open source project 'Fast-Drone-250' of Fei Gao's team.
   
   ---Positioning from the Motion Capture System: In the motion_cap_to_odom package.
