# Advanced Kinematics and EKF Localization Using IMU 🚀

## Overview
Explore a hands-on repository focused on practical scripts and concepts in mobile robotics and ROS 2. This collection emphasizes real-world applications and strengthens core skills in robotics. The content is enhanced with additional scripts to deepen understanding and practical skills in ROS 2.

## Key Learnings
### Kinematics 🧮
- Understanding the fundamentals of kinematics in mobile robotics.
- Implementing forward and inverse kinematics for a differential drive robot.
- Practical application of kinematic equations to control robot motion.

### Twist Message Handling 🔄
- Publishing and subscribing to ROS 2 `geometry_msgs/Twist` messages.
- Implementing publishers to control robot velocity.
- Creating subscribers to process and respond to Twist messages.
- Combining multiple subscribers and publishers into a single script for efficient data handling and robot control.

### Publishers and Subscribers 📡
- Gained an in-depth understanding of how ROS 2 publishers and subscribers facilitate communication between nodes.
- Developed scripts to experiment with message flow, including publishing velocity commands and subscribing to feedback data.
- Combined multiple subscribers and publishers for streamlined and modular robot control.

### URDF and Bot Design 🤖
- Designed a robot model using URDF (Unified Robot Description Format).
- Incorporated various meshes to represent robot components, ensuring an accurate and detailed visual representation.
- Utilized URDF to define joint structures, linkages, and physical properties of the robot.
- Verified the URDF design in simulation environments for functional testing.

### Differential Drive Controller 🛞
- The `diff_drive_controller` was used to smoothly translate velocity commands into wheel-specific actions, enabling reliable and precise robot movement.
- Configured the controller to interface seamlessly with the robot's hardware and ROS 2 topics.
- Utilized the controller to translate velocity commands into wheel-specific control inputs.

### Custom Simple Controller 🛠️
- Built a custom `simple_controller` in both Python and C++ to manage robot movement.
- Designed for direct and adaptable control over the robot's velocity.
- Added features to track velocity through `joint_states` for better feedback.
- Simulated real-world conditions with a noisy controller.
- Enabled pose estimation using wheel odometry and added an `odom` publisher for localization.

### Transform Publisher and Subscriber 🔀
- Added a transform publisher and subscriber in both Python and C++.
- Created a Python service to retrieve transform data, enabling efficient data handling and processing.

### Launch Files 🚀
- Launch files in ROS 2 are configuration scripts that simplify starting multiple nodes or processes together.
- Created files to make it easier to start the differential drive controller for the robot.
- These launch files also initialize the robot in Gazebo Ignition, with added features to automatically adjust for the detected ROS 2 and Gazebo versions.
- Integrated the noisy controller and pose estimation functionalities seamlessly into the launch sequence.
- Created a launch file to initialize the robot in Gazebo Ignition, with added functionality to automatically detect the Gazebo version based on the ROS 2 version.
- Enhanced compatibility and support for various ROS 2 versions through dynamic version checking in the launch files.
- Modified the launch file to integrate the noisy controller and additional features seamlessly.

## Enhancements 💡
To further explore and solidify the concepts, additional scripts were developed:

### C++ Scripts
1. **Twist Message Publisher**
   - Publishes linear and angular velocity commands.
   - Demonstrates efficient use of ROS 2 publishers in C++.

2. **Twist Message Subscriber**
   - Subscribes to Twist messages to monitor robot velocity.

3. **Combined Publisher and Subscriber**
   - Merges two subscribers and one publisher into a single script for streamlined functionality.

4. **Transform Publisher and Subscriber**
   - Publishes and subscribes to transforms, facilitating spatial awareness and coordinate transformations.

5. **Simple Controller**
   - Implements a basic controller for managing robot movement and velocity commands in C++.
   - Extracts velocity information using `joint_states` for feedback and control.
   - Publishes `odom` messages for enhanced localization.
   - Includes pose estimation using wheel odometry.

### Python Scripts
1. **Twist Message Publisher**
   - Implements a Python-based publisher for Twist messages.
   - Allows easy modification and testing of velocity commands.

2. **Twist Message Subscriber**
   - Subscribes to and processes incoming Twist messages.

3. **Combined Publisher and Subscriber**
   - Python script combining two subscribers and one publisher to manage robot velocity and control logic efficiently.

4. **Transform Publisher and Subscriber**
   - Publishes and subscribes to transforms for managing robot state and movement in a spatial context.

5. **Transform Service**
   - Python service for retrieving and processing transform data on demand.

6. **Simple Controller**
   - Python implementation of a custom controller for managing robot movement and velocity commands.
   - Extracts velocity information using `joint_states` for feedback and control.

### EKF Localization Using IMU
- Developed an Extended Kalman Filter (EKF) for localization.
- Used wheel encoder data for prediction and IMU data for correction or measurement.
- Created a new package named `bumperbot_localization` to implement EKF-based localization.

## Prerequisites 🛠️
- ROS 2 installed on your system.
- Basic understanding of ROS 2 concepts, including nodes, publishers, and subscribers.
- Familiarity with C++ and Python programming.

## How to Run ▶️
To get started with this project and explore its features, follow these steps:

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

5. Launch the controller or robot in Gazebo:
   - Launch the differential drive controller:
     ```bash
     ros2 launch bumperbot_controller controller.launch.py
     ```
   - Launch the robot in Gazebo Ignition:
     ```bash
     ros2 launch bumperbot_description gazebo.launch.py
     ```
   - Launch the EKF localization:
     ```bash
     ros2 launch bumperbot_localization ekf_localization.launch.py
     ```

## Future Plans 🌟
- Expanding advanced odometry and control topics.
- Adding more simulation environments for testing and visualization.

## Acknowledgments 🙌
- Contributions from various open-source robotics resources and communities.

## Contact 📬
For any questions or feedback, please reach out via:
- Email: devam3428@gmail.com
- GitHub: [Devam-032 GitHub Profile](https://github.com/Devam-032)

