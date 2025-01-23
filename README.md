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


