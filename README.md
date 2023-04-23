# TurtleBotics Reconnaisance Project 

## Introduction
In this project, we designed and implemented a completely autonomous system to perform reconnaissance in a simulated disaster environment. AprilTags were used as a proxy for victims in the simulated environment. The primary goals of this exercise were to map the unknown environment, localize the robot and accurately locate the AprilTags. A Turtlebot3 Burger was deployed to accomplish these tasks. The robot equipped with an array of sensors including LDS-01 LIDAR, a Raspberry Pi camera module V2 with Sony IMX219 8-megapixel sensor, and an IMU. Cartographer package was used to implement SLAM. To set up your turtlebot, follow the steps 3.1 to 3.3 on this [website](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

## Connect with the turtlebot
1. The robot is connected to the host PC via SSH. Ensure that the turtlebot and the host PC are connected over the same network.

2. The host PC should also act as the ROS master for the system. To configure your system as the ROS master, follow the steps as below:
    
   1. Open the bashrc on the **host PC**, and add the following lines:
   
      `export ROS_MASTER_URI=http://{IP ADDRESS OF HOST PC}:11311`

      `export ROS_HOSTNAME={IP ADDRESS OF HOST PC}`

   2. Open the bashrc on the **turtlebot**, and add the following lines:
   
        `ROS_MASTER_URI=http://{IP ADDRESS OF HOST PC}:11311`

        `ROS_HOSTNAME={IP ADDRESS OF TURTLEBOT}`

## Turtlebot Bringup
3. To launch LIDAR, IMU and wheel encoder along with all rostopics, run (on **turtlebot**):
   
   `roslaunch turtlebot3_bringup turtlebot3_robot.launch`

## Camera and AprilTag Detection
4. To launch the camera and the apriltag detection node, follow the steps as below (on **turtlebot**):
   
   1. Launch the camera
    
      `roslaunch raspicam_node turtlebotics_camera.launch camera_frame_id:={YOUR DESIRED FRAME ID} image_topic:={YOUR IMAGE TOPIC} camera_info_topic:={YOUR CAMERA INFO TOPIC}`

    2. Launch the apriltag detection node

       `roslaunch apriltag_ros continuous_detection.launch image_topic:={YOUR IMAGE TOPIC} camera_info_topic:={YOUR CAMERA INFO TOPIC}`

5. It is necessary that you add the camera link on your /tf topic as it is not added by default. To add your camera topic, run:
   
   `rosrun tf static_transform_publisher 0.03 0 0.1 0 0 0 base_link camera_frame 100`

## Navigation and SLAM
In this project, we have used cartographer to implement SLAM. It is necessary that you install cartographer package on your system by following the instructions on the official [website](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation)

6. Once your installation is complete, you need to update the `slam_methods` parameter to `cartographer` in `/opt/ros/noetic/share/turtlebot3_slam/launch/turtlebot3_slam.launch`

We have used explore_lite for planning and exploration of the area.

7. To install explore_lite, run:

   `sudo apt install ros-${ROS_DISTRO}-multirobot-map-mergeros-${ROS_DISTRO}-explore-lite`

Cartographer launches the move_base package by default which handles the path planning of the robot. 

