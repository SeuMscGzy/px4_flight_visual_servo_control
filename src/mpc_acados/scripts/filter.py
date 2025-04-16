#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Int32

#############################################
# 低通滤波器（简单一阶）
#############################################
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.last_output = 0.0

    def filter(self, input_val):
        output = self.alpha * input_val + (1 - self.alpha) * self.last_output
        self.last_output = output
        return output

#############################################
# 巴特沃斯二阶低通滤波器
#############################################
class ButterworthLowPassFilter:
    def __init__(self, fs, fc):
        self.fs = fs
        self.fc = fc
        # 存储前两次的输入与输出（初始值均为0）
        self.prevX = [0.0, 0.0]
        self.prevY = [0.0, 0.0]
        self.calculate_coefficients()

    def calculate_coefficients(self):
        ita = 1.0 / np.tan(np.pi * self.fc / self.fs)
        q = np.sqrt(2.0)
        a0_temp = 1.0 / (1.0 + q * ita + ita * ita)
        self.a0 = a0_temp
        self.a1 = 2 * a0_temp
        self.a2 = a0_temp
        self.b1 = 2.0 * a0_temp * (1 - ita * ita)
        self.b2 = a0_temp * (1 - q * ita + ita * ita)

    def filter(self, input_val):
        # 使用差分方程计算输出：
        # y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
        output = (self.a0 * input_val + 
                  self.a1 * self.prevX[0] + 
                  self.a2 * self.prevX[1] - 
                  self.b1 * self.prevY[0] - 
                  self.b2 * self.prevY[1])
        # 更新历史记录
        self.prevX[1] = self.prevX[0]
        self.prevX[0] = input_val
        self.prevY[1] = self.prevY[0]
        self.prevY[0] = output
        return output

#############################################
# 控制器类（转换自 C++ 代码）
#############################################
class MyController:
    def __init__(self):
        # 初始化控制输入（3维向量）
        self.u = np.zeros(3)

        # 实时状态与导数
        self.x_real = 0.0
        self.x_real_last = 0.0
        self.y_real = 0.0
        self.y_real_last = 0.0
        self.z_real = 0.0
        self.z_real_last = 0.0

        self.x_real_derivative = 0.0
        self.y_real_derivative = 0.0
        self.z_real_derivative = 0.0

        self.x_filtered_deri = 0.0
        self.y_filtered_deri = 0.0
        self.z_filtered_deri = 0.0

        # 时间记录（单位：秒）
        self.time_now = rospy.Time.now().to_sec()
        self.time_last = self.time_now
        self.time_pass = 0.0

        # 状态标志
        self.first_time_in_fun = True
        self.loss_target = True  # 初始状态与 C++ 中一致
        self.loss_or_not_ = 1.0

        # 构造低通滤波器（图像信号滤波）
        self.filter_for_imgx = LowPassFilter(0.97)
        self.filter_for_imgy = LowPassFilter(0.97)
        self.filter_for_imgz = LowPassFilter(0.97)

        # 构造巴特沃斯低通滤波器（用于导数滤波），采样率为20Hz，截止频率为2.7Hz
        self.filter_4_for_derix = ButterworthLowPassFilter(20, 2.7)
        self.filter_4_for_deriy = ButterworthLowPassFilter(20, 2.7)
        self.filter_4_for_deriz = ButterworthLowPassFilter(20, 2.7)

        # PX4 状态：初始值设为1
        self.px4_state = 1

        # ROS 订阅者
        self.px4_state_sub = rospy.Subscriber("/px4_state_pub", Int32, self.state_callback, queue_size=1)
        self.sub = rospy.Subscriber("/point_with_fixed_delay", Float64MultiArray, self.callback, queue_size=1)

        # 如果需要，可增加其他订阅（例如 ground truth 的速度、位置）
        # 例如：
        # self.ground_truth_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.ground_truth_callback, queue_size=10)
        # self.ground_truth_pose_sub = rospy.Subscriber("/vrpn_client_node/MCServer/5/pose", PoseStamped, self.ground_truth_pose_callback, queue_size=10)

        rospy.loginfo("MyController initialized.")

    def callback(self, msg):
        data = msg.data
        if len(data) < 5:
            rospy.logerr("接收到的数据长度不足：{}".format(len(data)))
            return

        # 取出前3个数据作为位置信息
        xyz_1 = np.array([data[0], data[1], data[2]])
        loss_or_not_val = data[4]
        self.cal_single_axis_ctrl_input(xyz_1, loss_or_not_val)

    def cal_single_axis_ctrl_input(self, xyz_1, loss_or_not):
        """
        计算单轴（这里扩展为三轴）的控制输入：
          - 使用图像信息滤波器对测量到的位置信息进行滤波
          - 计算与上一时刻的差分获得导数
          - 利用巴特沃斯滤波器对导数进行滤波
          - 调用 function() 做进一步处理
        """
        self.loss_or_not_ = loss_or_not
        # 更新三轴状态（滤波后的）
        self.x_real = self.filter_for_imgx.filter(xyz_1[0])
        self.y_real = self.filter_for_imgy.filter(xyz_1[1])
        self.z_real = self.filter_for_imgz.filter(xyz_1[2])

        self.time_now = rospy.Time.now().to_sec()
        self.time_pass = self.time_now - self.time_last
        if self.time_pass <= 0:
            self.time_pass = 0.05  # 防止除零

        # 计算导数
        self.x_real_derivative = (self.x_real - self.x_real_last) / self.time_pass
        self.y_real_derivative = (self.y_real - self.y_real_last) / self.time_pass
        self.z_real_derivative = (self.z_real - self.z_real_last) / self.time_pass

        # 利用巴特沃斯滤波器滤波导数
        self.x_filtered_deri = self.filter_4_for_derix.filter(self.x_real_derivative)
        self.y_filtered_deri = self.filter_4_for_deriy.filter(self.y_real_derivative)
        self.z_filtered_deri = self.filter_4_for_deriz.filter(self.z_real_derivative)

        # 调用处理函数（例如用于控制切换或重置状态）
        self.function(self.loss_or_not_)

        # 更新上一时刻的状态与时间
        self.x_real_last = self.x_real
        self.y_real_last = self.y_real
        self.z_real_last = self.z_real
        self.time_last = self.time_now

    def function(self, loss_or_not):
        if loss_or_not == 1 and not self.loss_target:
            self.loss_target = True
        if loss_or_not == 0 and self.loss_target:
            self.loss_target = False
            self.first_time_in_fun = True

        if self.first_time_in_fun:
            self.time_pass = 0.05
            self.x_filtered_deri = 0.0
            self.y_filtered_deri = 0.0
            self.z_filtered_deri = 0.0
            self.first_time_in_fun = False
        else:
            # else 分支中可加入其他处理，此处与 C++ 版本一致，不做其他处理
            pass

    def state_callback(self, msg):
        """
        处理 /px4_state_pub 话题传入的 Int32 消息：
          - 更新 px4_state，当不处于命令模式（值不为3）时，将控制输入置零
        """
        self.px4_state = msg.data
        if msg.data != 3:
            self.u = np.zeros(3)

    def spin(self):
        rospy.spin()
