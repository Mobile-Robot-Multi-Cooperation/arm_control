<!--
 * @Author: WYD 
 * @Mail: 405131298@qq.com
 * @Description: 
-->
UR5-OCS2 controller
===
## Dependend

1. ocs2_ros2
2. xacro
3. ros2-control ros2-controller
4. localization_msgs
5. Universal_Robots_ROS2_Driver
6. 

## Demo
``` sh
colcon build 
source install/setup.bash
```

### Gazebo
``` sh
ros2 launch ur5_description bringup_gazebo.launch.py
ros2 run rqt_controller_manager rqt_controller_manager
```

### Real
``` sh
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.3.20
Open ExternalControl_URCap
ros2 launch ur5_description bringup_real.launch.py ur_type:=ur5 robot_ip:=192.168.3.20
ros2 run rqt_controller_manager rqt_controller_manager
```

