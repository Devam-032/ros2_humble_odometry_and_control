# Kinematics and Twist Message Practice

## Overview
This repository contains implementations and practice scripts based on key learnings in mobile robotics and ROS 2. The content is enhanced with additional scripts to deepen understanding and practical skills in ROS 2.

## Key Learnings
### Kinematics
- Understanding the fundamentals of kinematics in mobile robotics.
- Implementing forward and inverse kinematics for a differential drive robot.
- Practical application of kinematic equations to control robot motion.

### Twist Message Handling
- Publishing and subscribing to ROS 2 `geometry_msgs/Twist` messages.
- Implementing publishers to control robot velocity.
- Creating subscribers to process and respond to Twist messages.
- Combining multiple subscribers and publishers into a single script for efficient data handling and robot control.

### Publishers and Subscribers
- Gained an in-depth understanding of how ROS 2 publishers and subscribers facilitate communication between nodes.
- Developed scripts to experiment with message flow, including publishing velocity commands and subscribing to feedback data.
- Practiced combining multiple subscribers and publishers for streamlined and modular robot control.

### URDF and Bot Design
- Designed a robot model using URDF (Unified Robot Description Format).
- Incorporated various meshes to represent robot components, ensuring an accurate and detailed visual representation.
- Utilized URDF to define joint structures, linkages, and physical properties of the robot.
- Verified the URDF design in simulation environments for functional testing.

### Differential Drive Controller
- Implemented the `diff_drive_controller` to enable precise control of the differential drive robot.
- Configured the controller to interface seamlessly with the robot's hardware and ROS 2 topics.
- Used the controller to translate velocity commands into wheel-specific control inputs.

## Practice Enhancements
To further explore and solidify the concepts, additional practice scripts were developed:

### C++ Scripts
1. **Twist Message Publisher**
   - Publishes linear and angular velocity commands.
   - Demonstrates efficient use of ROS 2 publishers in C++.

2. **Twist Message Subscriber**
   - Subscribes to Twist messages to monitor robot velocity.

3. **Combined Publisher and Subscriber**
   - Merges two subscribers and one publisher into a single script for streamlined functionality.

### Python Scripts
1. **Twist Message Publisher**
   - Implements a Python-based publisher for Twist messages.
   - Allows easy modification and testing of velocity commands.

2. **Twist Message Subscriber**
   - Subscribes to and processes incoming Twist messages.

3. **Combined Publisher and Subscriber**
   - Python script combining two subscribers and one publisher to manage robot velocity and control logic efficiently.

## Prerequisites
- ROS 2 installed on your system.
- Basic understanding of ROS 2 concepts, including nodes, publishers, and subscribers.
- Familiarity with C++ and Python programming.

## How to Run
1. Clone this repository:
   ```bash
   git clone https://github.com/Devam-032/ros2_humble_odometry_and_control/tree/main
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```
4. Run the desired script:
   - For C++:
     ```bash
     ros2 run bumperbot_cpp_examples <executable_name>
     ```
   - For Python:
     ```bash
     ros2 run bumperbot_py_examples <script_name>
     ```

## Future Plans
- Implementing advanced odometry and control topics.
- Adding simulation environments for testing and visualization.

## Acknowledgments
- Contributions from various open-source robotics resources and communities.

## Contact
For any questions or feedback, please reach out via:
- Email: devam3428@gmail.com
- GitHub: [Devam-032 GitHub Profile](https://github.com/Devam-032)

