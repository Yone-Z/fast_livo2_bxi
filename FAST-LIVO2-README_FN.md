```bash
  程序copy到新电脑时，需要重新编译livox_driver2 && fast-livo2 
```

# 相机启动：下载对应的包
```bash
# 正常启动
  source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py

# 只启动RGB 不要深度、红外
  source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py enable_depth:=false enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=1280x720x15
  source /opt/ros/humble/setup.bash && source install/setup.bash && nohup ros2 launch realsense2_camera rs_launch.py enable_depth:=false enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=1280x720x15 > sensor.log &   #后台启动

# 以更低分辨率和帧率启动 不能用
  source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x15 rgb_camera.color_profile:=640x480x15
```

# 激光雷达mid360启动

<!-- 电脑ip需设置为：    "point_data_ip": "192.168.2.50",
      雷达ip预设为：         "ip" : "192.168.2.202", -->

```bash
# rviz启动 不要用
  source install/setup.bash && ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# rviz不启动(建图是要选这种启动方式)
  source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py
  source install/setup.bash && nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py > lidar.log & #后台启动

```

# fast-livo2建图
```bash
[camera_init坐标系若想始终跟重力对其，只需要配置yaml文件中的： uav： “gravity_align_en: true”]

  colcon build --packages-select fast_livo   #单独编译fast-livo2
  source install/setup.bash && ros2 launch fast_livo mapping_mid360_realsense2.launch.py
  source install/setup.bash && nohup ros2 launch fast_livo mapping_mid360_realsense2.launch.py > livox2.log &   #后台启动
```

# 录制rosbag:
```bash
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 bag record -a
# 播放rosbag:
  ros2 bag play <bag_name> --loop
# 修复rosbag
  ros2 bag reindex <bag_name>
## 查看时间戳是否对齐：
  ros2 run rqt_bag rqt_bag 
```
# 高程图
```bash
  source install/setup.bash 
# ros2 run elevation_map_node elevation_map_generator
  source install/setup.bash && python3 src/elevation_map_node/src/elevation_map_generator4.py
```


# 查看内存占用
```bash
# %MEM ：进程使用的物理内存百分比,建图是，内存涨到50%则会卡顿，大约10min左右
  top 命令查看

# 

```

<!-- # 时间同步
source install/setup.bash 
ros2 launch fast_livo mapping_mid360_realsense_sync.launch.py

# 时间同步_rviz启动
source install/setup.bash 
ros2 launch livox_ros_driver2 complete_mapping_sync_launch.py


ros2 interface show nav_msgs/msg/Odometry
ros2 topic echo /aft_mapped_to_init --once 

# 时间同步
  source install/setup.bash
  ros2 launch livox_ros_driver2 msg_MID360_sync_launch.py-->