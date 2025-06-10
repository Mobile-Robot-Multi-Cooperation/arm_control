<!--
 * @Author: WYD 
 * @Mail: 405131298@qq.com
 * @Description: 
-->
Moying-OCS2 controller
===
## Dependend

1. ocs2_ros2
2. xacro
3. ros2-control ros2-controller
4. localization_msgs
5. moying_ros2

## Demo
``` sh
colcon build 
source install/setup.bash
```

### Gazebo
``` sh
ros2 launch moying_arm_bringup arm.launch.py
ros2 run rqt_controller_manager rqt_controller_manager
```

