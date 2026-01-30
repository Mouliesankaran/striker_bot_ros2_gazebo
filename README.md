# striker_bot_ros2_gazebo
ROS 2 based mobile robot simulation using Gazebo. The project includes a XACRO-based robot description and a launch file to spawn the robot in Gazebo with robot_state_publisher.

This project contains a ROS 2 package to simulate a custom mobile robot (**Striker Bot**) in Gazebo.
The robot is described using **URDF/XACRO** and spawned into Gazebo using a ROS 2 launch file.

---

## 📦 Package Name
`striker_bot_description`

---

## 🛠️ Features
- XACRO-based robot description
- Gazebo simulation support
- Robot State Publisher integration
- Automatic robot spawning in Gazebo
- Uses ROS 2 standard Gazebo launch

---

## 📁 Project Structure

striker_bot_description/
├── launch/
│ └── gazebo.launch.py
├── urdf/
│ └── striker_bot.urdf.xacro
├── package.xml
└── CMakeLists.txt


---

## 🚀 How to Run

### 1️⃣ Prerequisites
Make sure you have:
- ROS 2 (Humble / Foxy / Iron)
- Gazebo
- gazebo_ros package

### 2️⃣ Build the Workspace
```bash
colcon build
source install/setup.bash

3️⃣ Launch the Simulation

ros2 launch striker_bot_description gazebo.launch.py

This will:

    Start Gazebo

    Load the robot description

    Publish TF using robot_state_publisher

    Spawn the Striker Bot into Gazebo
