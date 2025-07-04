# probabilisticLab
This ROS 2 project implements and compares three localization algorithms for 2D robot localization in a Gazebo simulation using TurtleBot3:

- Kalman Filter (KF)
- Extended Kalman Filter (EKF)
- Particle Filter (PF)

Each filter runs in its own C++ node and estimates the robot's position based on odometry and IMU data. Navigation goals are issued to test performance under different movement patterns.

Project Structure
-----------------
localization_filters/
├── src/                        # C++ source code
│   ├── kalman_filter.cpp       - Kalman Filter node
│   ├── ekf.cpp                 - Extended Kalman Filter node
│   ├── particle_filter.cpp     - Particle Filter node
│   ├── navigator.cpp           - Sends waypoint goals
│   ├── drive_circle.cpp        - Drives robot in a circular path
│   ├── drive_eight.cpp         - Drives robot in a figure-eight path
│   ├── pose_logger.cpp         - Logs estimated and true poses
├── launch/
│   ├── start.launch.py         - Launches full simulation (Gazebo, RViz, filters, navigation)
│   ├── turtlebot.launch.py     - Launches TurtleBot3 and simulation
├── config/
│   ├── nav2_params.yaml        - Navigation stack config
│   ├── waypoints.yaml          - Defines multiple navigation goals
│   ├── map.yaml / map.pgm      - Static map for RViz and localization
├── comparison_*.csv            - Evaluation results for different trajectories
├── final_comparison_*.csv      - Final evaluation results
├── Comparison_1.png            - Example result plot
├── CMakeLists.txt, package.xml - Build and package files

How to Run
----------
1. Build the workspace:
   $ colcon build
   $ source install/setup.bash

2. Launch full system (map, robot, filters, goals, RViz):
   $ ros2 launch localization_filters turtlebot.launch.py
   
   
I didn't get nav2 to run, so i just make the robot move either by teleop or the drive_*pattern*.cpp exec. files.
The cmd_vel watchdog makes sure, that the robot stops when any of those get terminated.
The logger creates a .csv file in the workspace so that the three filters can be compared.
