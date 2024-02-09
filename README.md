# Autonomous Quadcopter Position Control using Visual Servoing Simulation in Gazebo
My second phase for my undergraduate thesis which is simulation the vision part, which is the crucial part on my undergraduate thesis. A Quadcopter navigation system using the visual sensor as it's main navigation sensor system to reach it's destination. This package program is a quadrotor simulation program forked from sjtu_drone(https://github.com/NovoG93/sjtu_drone), developed using ROS 2 Foxy - Gazebo Classic 11 Simulation.

## Downloading
In this simulation, I have divide it into two section, which is capturing the visual and control, where each of them from two node that is the image_subscriber and camera_control. So the thing you wanna do is first make your own workspace and put all the code in this github to the src folder. 

```sh
$ mkdir yourworkspace/src
$ cd yourworkspace/src/
$ git clone https://github.com/WallNutss/EE234899-Quadcopter-Gazebo-VisualServoing.git
```
To use the .world file in the simulation, make sure to install the common gazebo models, for more see the Readme in sjtu_drone_description.

## How to Run (ROS 2 Foxy)
1. After you download the code and install Gazebo 11 for Foxy, you can start gazebo and spawn drone:
```sh
$ ros2 launch  sjtu_drone_bringup sjtu_drone_bringup.launch.py
```
2. Start the camera capture node and takeoff the drone
```sh
$ ros2 run image_subscriber pose_display
```
3. Start IBVS for the drone to go
If you want to choose IBVS-PID
```sh
$ ros2 run camera_control ibvs-pid
```
If you want to choose IBVS-ISMC
```sh
$ ros2 run camera_control ibvs-smc
```
3. Land drone:
```sh
$ ros2 topic pub /drone/land std_msgs/msg/Empty {} --once
```


### Camera Capture

### Image Based Visual Servoing

## System Architecture

## Contact
Muhammad Juniarto

Email : wallnuts73@gmail.com
