#!/bin/bash

# 启动Gazebo环境，后台执行，保存PID
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world &
pid_gazebo=$!
echo "Gazebo PID: $pid_gazebo"
sleep 2

# 启动PX4仿真
make px4_sitl none_iris &
pid_px4=$!
echo "PX4 PID: $pid_px4"
sleep 2

# 启动MAVROS
roslaunch mavros px4.launch fcu_url:='udp://:14540@' &
pid_mavros=$!
echo "MAVROS PID: $pid_mavros"
sleep 2

# 启动4个MAVCMD命令
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 &
pid_mavcmd1=$!
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 &
pid_mavcmd2=$!
rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0 &
pid_mavcmd3=$!
rosrun mavros mavcmd long 511 36 5000 0 0 0 0 0 &
pid_mavcmd4=$!
echo "MAVCMD PIDs: $pid_mavcmd1, $pid_mavcmd2, $pid_mavcmd3, $pid_mavcmd4"

echo "全部启动完成。按 Ctrl+C 关闭所有进程。"

# 捕捉 Ctrl+C 关闭所有子进程
cleanup() {
  echo "关闭所有仿真进程..."
  kill $pid_gazebo $pid_px4 $pid_mavros $pid_mavcmd1 $pid_mavcmd2 $pid_mavcmd3 $pid_mavcmd4 2>/dev/null
  echo "关闭完成。"
  exit 0
}

trap cleanup SIGINT

# 等待所有后台进程，保持脚本运行
wait
