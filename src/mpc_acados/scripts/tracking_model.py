from casadi import *
import numpy as np
import types

def tracking_model():
    # 创建一个 SimpleNamespace 存储模型信息
    model = types.SimpleNamespace()
    model_name = "tracking_model"

    # ------------------------------
    # 1. 定义状态和控制
    # ------------------------------
    # 状态：三维位置和速度，每个都是3维向量
    relative_pos = MX.sym("relative_pos", 3)  # 三维位置
    relative_vel = MX.sym("relative_pos", 3)  # 三维速度
    x = vertcat(relative_pos, relative_vel)   # 状态向量，共6个元素

    # 控制：三维加速度输入
    u = MX.sym("u", 3)      # 控制向量

    # ------------------------------
    # 2. 定义状态导数符号
    # ------------------------------
    # 为构造隐式模型表达式，定义状态导数（位置和速度导数均为3维）
    relative_pos_dot = MX.sym("relative_pos_dot", 3)
    relative_vel_dot = MX.sym("relative_vel_dot", 3)
    xdot = vertcat(relative_pos_dot, relative_vel_dot)

    # ------------------------------
    # 3. 无代数变量和模型参数
    # ------------------------------
    z = vertcat([])   # 代数变量为空
    p = vertcat([])   # 模型参数为空

    # ------------------------------
    # 4. 定义动力学表达式
    # ------------------------------
    # 显式动力学表达式：
    #   dot(pos) = vel
    #   dot(vel) = u
    f_expl = vertcat(
        relative_vel,  # 位置导数等于速度
        u     # 速度导数等于控制输入（加速度）
    )
    # 隐式表达式： xdot - f_expl = 0
    f_impl_expr = xdot - f_expl

    # ------------------------------
    # 5. 设置控制输入边界
    # ------------------------------
    # 例如限制控制输入在 [-10,10] 区间内，对每个轴都设定同样的限制
    model.u_min = np.array([-4, -4, -4])
    model.u_max = np.array([4, 4, 4])

    # ------------------------------
    # 6. 定义初始状态
    # ------------------------------
    # 默认初始状态均为0：[pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
    model.x0 = np.zeros(6)

    # ------------------------------
    # 7. 构造模型结构体
    # ------------------------------
    model.f_expl_expr = f_expl      # 显式动力学表达式
    model.f_impl_expr = f_impl_expr  # 隐式动力学表达式（用于校验或求解器使用）
    model.x = x                     # 状态向量
    model.xdot = xdot               # 状态导数符号
    model.u = u                     # 控制向量
    model.z = z                     # 代数变量
    model.p = p                     # 参数
    model.name = model_name         # 模型名称

    return model