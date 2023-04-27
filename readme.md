# 安装依赖
`sudo apt install ros-noetic-moveit-ros-visualization`

`sudo apt install ros-noetic-joint-trajectory-controller`

`pip install pyrealsense2`


# 运行
run simulation environment 

`roslaunch jaka_minicobo_moveit_config demo_mini_cam.launch`

run robot control

`rosrun jaka_sim_env move_jaka.py`

## 总体代码
可以运行robot_scan.py从而实现从读取点云数据到轨迹规划部分（仿真环境）

