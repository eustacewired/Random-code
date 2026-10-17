# Building - either
```
rm -rf build install log && colcon build --symlink-install && source install/setup.bash
```

# Virtual Teleop + SLAM
**Terminal 1 (Gazebo + robot)**
This launches the robot in Gazebo with use_sim_time:=true baked in. Build and save a map
```
source install/setup.bash && ros2 launch auracle_bringup launch_sim.launch.py
```
Wait for it to settle — Gazebo window opens, spawner sequence finishes diff_cont → joint_broad → imu_broadcaster, one after another now

To confirm in a separate terminal
```
ros2 control list_controllers          # diff_cont must show "active". If any inactive, launch with ros2 run controller_manager spawner diff_cont
```
**Terminal 2 (SLAM, NAV2, Rviz)**
```
source install/setup.bash && ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=true
```
**Terminal 3 (Teleop)**
```
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```

**Terminal 4 (Map saving)**
This writes map_1.yaml + map_1.pgm into config/
```
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/Documents/auracle_ws/src/auracle_bringup/config/map_1
```
Add map_server block to nav2_params.yaml
```
map_server:
  ros__parameters:
    yaml_filename: ""
```

## Navigation

**Terminal 1 (simulation)**
```
source install/setup.bash && ros2 launch auracle_bringup launch_sim.launch.py
```

**Terminal 2 (AMCL + Nav2 + RViz, against your saved map)**
```
source install/setup.bash&& ros2 launch auracle_bringup localization_nav_rviz.launch.py use_sim_time:=true map:=$(pwd)/src/auracle_bringup/config/map_1.yaml
```

In RViz, check that:
the global costmap now covers the whole map;
the particle cloud is a tight cluster around the robot;
there are no TF errors in the Displays panel.
Click Nav2 Goal in the toolbar, then click-and-drag on a free (grey) spot of the map. The click sets the position and the drag sets the heading.
A green line (GlobalPlan) should appear and the robot should start driving in Gazebo. Navigation goes through cmd_vel_nav into twist_mux at priority 40, which is wired correctly.
Don’t press keys in the teleop window while a goal is running, because joystick and teleop input has priority 100 and overrides Nav2.


# Real robot (Pi 5 + Arduino Nano)

## Flashing the Nano from WSL and platformIO. (See platform.ini for ArduinoIDE)
PowerShell as admin, once per device:
```
usbipd list                      # Nano = FTDI 0403:6001, e.g. BUSID 1-2
usbipd bind --busid=1-2
```
Every time you replug:
```
usbipd attach --wsl --busid=1-2
```
In WSL:
```
ls /dev/ttyUSB*                  # FTDI shows up as /dev/ttyUSB0
cd firmware/auracle_firmware
pio run -t upload --upload-port /dev/ttyUSB0
```
To flash from the Pi instead, use the same command with `--upload-port /dev/arduino`. Close ROS first, because only one program can hold the port.

## Installations 
```
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-robot-localization   ros-jazzy-twist-mux ros-jazzy-twist-stamper ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox   ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher libserial-dev
```


## Adding udev rules
```
#check if rules file exists
ls /etc/udev/rules.d/99-auracle.rules
#if missing, copy from workspace
sudo cp src/auracle_bringup/config/99-auracle.rules /etc/udev/rules.d/
```
Reload and trigger the udev rules to generate the symlinks. Physical connection: Both on blue ports, lidar on top of arduino
```
sudo udevadm control --reload-rules
sudo udevadm trigger
#Verify the symlinks now exist and point to the correct ttyUSB ports
ls -l /dev/arduino /dev/rplidar
#ensure your user account is in the dialout group to allow serial access:
udo usermod -aG dialout $USER
#log out and log back into your Ubuntu session for the group changes to take effect.
sudo reboot
```

**On Pi Terminal: robot (hardware interface, controllers, lidar, EKF)**
```
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py namespace:=auracle
```
If lidar not connected, skip it:
```
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py use_lidar:=false namespace:=auracle
```
Confirm topics and services are live with `ros2 topic list` and `ros2 service list`

**Terminal 2: keyboard teleop**
```
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy -r __ns:=/auracle
```
**Joystick (optional)**
With controller paired over bluetooth on the laptop, this has to be launched on the laptop
```
ros2 launch auracle_bringup joystick.launch.py use_sim_time:=false namespace:=auracle
```

**SLAM + Nav2 + RViz** (run on the laptop, same `ROS_DOMAIN_ID`)
```
source install/setup.bash && ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=false namespace:=auracle
```
