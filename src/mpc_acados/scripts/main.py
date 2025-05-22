#!/usr/bin/env python
import rospy
import numpy as np
from filter import MyController
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from tracking_model import tracking_model
from acados_settings import acados_settings
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import TwistStamped, PoseStamped
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
import time
from openpyxl import Workbook

class AcadosTrackingNode:
    def __init__(self):
        self.filter1 = MyController()
        # Set acados parameters (prediction horizon Tf and number of discrete nodes N, adjust as needed)
        Tf = 1.0   # prediction horizon (seconds)
        N  = 20    # number of discretization nodes
        
        # Call acados_settings to get the model object and OCP solver
        self.model, self.acados_solver = acados_settings(Tf, N)

        # Initialize publishers for ROS topics
        # Publisher for control input topic: message type quadrotor_msgs/PositionCommand
        self.control_pub = rospy.Publisher("/acc_cmd", PositionCommand, queue_size=1)
        # Publisher for x-axis input message: message type std_msgs/Float64
        # self.x_pub = rospy.Publisher("/input_x_axis", Float64, queue_size=1)
        # Publisher for estimated state (hat_x) message: message type std_msgs/Float64MultiArray
        self.pub_hat_x = rospy.Publisher("/hat_x_topic", Float64MultiArray, queue_size=1)

        # Timer to periodically call state update and control computation
        self.state_timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        # If needed, can also subscribe to other messages (e.g., ground truth), example below:
        self.ground_truth_sub = rospy.Subscriber(
            "/mavros/local_position/velocity_local",
            TwistStamped, self.ground_truth_callback, queue_size=1)
        self.ground_truth_pose_sub = rospy.Subscriber(
            "/vrpn_client_node/cf1/0/pose",
            PoseStamped, self.ground_truth_pose_callback, queue_size=1)

        self.x0 = np.zeros(self.model.x.size()[0])
        self.u0 = np.zeros(self.model.u.size()[0])

        # Initialize ground truth related variables
        self.ground_truth_first_deri_x = 0.0
        self.ground_truth_first_deri_y = 0.0
        self.ground_truth_first_deri_z = 0.0

        self.ground_truth_x = 0.0
        self.ground_truth_y = 0.0
        self.ground_truth_z = 0.0

        rospy.loginfo("Acados Tracking Node init。")

    def publish_control_and_hat_x(self, des_yaw=0.0):
        # Publish quadrotor_msgs/PositionCommand control message
        acc_msg = PositionCommand()
        acc_msg.position.x = 0
        acc_msg.position.y = 0
        acc_msg.position.z = 0
        acc_msg.velocity.x = 0
        acc_msg.velocity.y = 0
        acc_msg.velocity.z = 0
        acc_msg.acceleration.x = self.u0[0]
        acc_msg.acceleration.y = self.u0[1]
        acc_msg.acceleration.z = self.u0[2]
        acc_msg.jerk.x = 0
        acc_msg.jerk.y = 0
        acc_msg.jerk.z = 0
        acc_msg.yaw = des_yaw
        acc_msg.yaw_dot = 0
        acc_msg.header.frame_id = "world"
        acc_msg.header.stamp = rospy.Time.now()
        self.control_pub.publish(acc_msg)

        msgx = Float64()
        msgx.data = self.u0[0]
        # self.x_pub.publish(msgx)

        # Publish hat_x message (Float64MultiArray type)
        msg1 = Float64MultiArray()
        msg1.data = [0.0] * 27  # Initialize with 27 zeros

        # X-axis data
        msg1.data[2] = self.u0[0]
        msg1.data[3] = self.x0[0]
        msg1.data[4] = -self.ground_truth_first_deri_x
        msg1.data[5] = self.ground_truth_first_deri_x
        msg1.data[6] = self.x0[3]
        msg1.data[7] = self.ground_truth_first_deri_x
        msg1.data[8] = self.ground_truth_x

        # Y-axis data
        msg1.data[11] = self.u0[1]
        msg1.data[12] = self.x0[1]
        msg1.data[13] = -self.ground_truth_first_deri_y
        msg1.data[14] = self.ground_truth_first_deri_y
        msg1.data[15] = self.x0[4]
        msg1.data[16] = self.ground_truth_first_deri_y
        msg1.data[17] = self.ground_truth_y

        # Z-axis data
        msg1.data[20] = self.u0[2]
        msg1.data[21] = self.x0[2] + 0.1
        msg1.data[22] = -self.ground_truth_first_deri_z
        msg1.data[23] = self.ground_truth_first_deri_z
        msg1.data[24] = self.x0[5]
        msg1.data[25] = self.ground_truth_first_deri_z
        msg1.data[26] = self.ground_truth_z

        self.pub_hat_x.publish(msg1)

    def timer_callback(self, event):
        # Update initial state x0 based on filtered measurements
        self.x0 = np.array([
            self.filter1.x_real - 0.9,
            self.filter1.y_real,
            self.filter1.z_real - 0.1,
            self.filter1.x_filtered_deri,
            self.filter1.y_filtered_deri,
            self.filter1.z_filtered_deri
        ])
        # Solve the OCP by updating x0 internally in the solver
        self.u0 = self.acados_solver.solve_for_x0(self.x0)
        self.publish_control_and_hat_x()

    def ground_truth_callback(self, msg):
        # Update ground truth velocities
        self.ground_truth_first_deri_x = msg.twist.linear.x
        self.ground_truth_first_deri_y = msg.twist.linear.y
        self.ground_truth_first_deri_z = msg.twist.linear.z
        # rospy.loginfo("Ground truth velocity: x: {:.3f}, y: {:.3f}, z: {:.3f}".format(
        #     self.ground_truth_first_deri_x,
        #     self.ground_truth_first_deri_y,
        #     self.ground_truth_first_deri_z
        # ))

    def ground_truth_pose_callback(self, msg):
        # Update ground truth position
        self.ground_truth_x = msg.pose.position.x
        self.ground_truth_y = msg.pose.position.y
        self.ground_truth_z = msg.pose.position.z
        # rospy.loginfo("Ground truth pose: x: {:.3f}, y: {:.3f}, z: {:.3f}".format(
        #     self.ground_truth_x,
        #     self.ground_truth_y,
        #     self.ground_truth_z
        # ))

def main():
    rospy.init_node("acados_tracking_node", anonymous=True)
    node = AcadosTrackingNode()
    rospy.spin()

if __name__ == '__main__':
    main()
