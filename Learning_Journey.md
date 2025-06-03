# ROS 2 Publisher/Subscriber Project with Docker on AWS EC2

This project is part of a hands-on learning journey in robotics using ROS 2 (Robot Operating System), Docker, and AWS EC2. The objective is to demonstrate basic ROS 2 concepts including nodes, publisher/subscriber communication, and setup on a lean, reproducible cloud-hosted development environment.

---

## 📦 Project Name
**ros2_pubsub_package**

---

## 🚀 What You Learned

### 1. **ROS 2 Fundamentals**
- ROS 2 is a framework for building robot applications using nodes that communicate via topics.
- Nodes can be written in Python or C++.
- Topics are the channels used to send (publish) or receive (subscribe) messages.

### 2. **ROS 2 Nodes**
- Publisher and subscriber nodes created in Python using `rclpy` (ROS 2 Python client library).

### 3. **ROS 2 Workspace Structure**
```
ros2_pubsub_package/
├── ros2_pubsub_package/
│   ├── __init__.py
│   ├── publisher_node.py
│   └── subscriber_node.py
├── resource/
│   └── ros2_pubsub_package
├── setup.py
├── setup.cfg
├── package.xml
└── CMakeLists.txt
```

### 4. **Building ROS 2 Package**
```bash
colcon build
source install/setup.bash
```

### 5. **Running Publisher & Subscriber**
```bash
ros2 run ros2_pubsub_package publisher_node
ros2 run ros2_pubsub_package subscriber_node
```

Use separate terminal windows or tmux panes to run each.

### 6. **Using Docker**
- Used Docker image `ros:humble` for a consistent ROS 2 environment.
- Docker container set up with volume mounts to host project directory.

### 7. **AWS EC2 Setup**
- Launched Ubuntu instance (free-tier `t2.micro`).
- Installed Docker, Git, and configured access via SSH.
- Created shell scripts to automate EC2 start/stop/status using AWS CLI.
- Allocated Elastic IP for persistent connection.

### 8. **Version Control & Portability**
- This project will be committed to GitHub as a portfolio piece for showcasing ROS 2 fundamentals.

---

## 🐳 Docker Usage
```bash
docker run -it --rm \
  --name ros2_pubsub \
  -v ~/ros2_pubsub_package:/ros2_ws/src/ros2_pubsub_package \
  ros:humble
```

Inside Docker:
```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

---

## 🛠️ To Do
- Add RViz2 or Gazebo simulation.
- Expand with timers, parameters, or lifecycle nodes.
- Build robotics application scenarios (e.g., TurtleBot simulation).

---

## 🧠 Goal
To use this project as a foundational step into ROS 2, Docker, cloud robotics, and simulation—building toward autonomous robotics and real-world deployment.

---

## 📁 GitHub Repo
```
ros2_pubsub_project/
├── ros2_pubsub_package/     # ROS 2 package
├── docker-compose.yml       # Optional for more complex setups
├── start-ec2.sh             # Script to start EC2
├── stop-ec2.sh              # Script to stop EC2
├── status-ec2.sh            # Script to check EC2 status
├── README.md                # This file
```

