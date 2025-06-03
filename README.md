## ROS 2 (Humble) on AWS EC2 using Docker - Python Publisher/Subscriber Demo

Welcome! This repository helps you get started with **ROS 2 Humble** using **Python** on a lean, low-cost **AWS EC2 instance** via **Docker**. It's designed as a beginner-friendly project for your robotics and cloud portfolio.

---

## 🚀 Overview

This project shows you how to:
- Launch and control an EC2 instance from CLI
- Set up Docker and ROS 2 (Humble)
- Build a Python-based ROS 2 package
- Run a basic publisher/subscriber node

---

## 💻 Project Structure

```
ros2_ec2_docker_setup/
├── ros2_ws/
│   └── src/
│       └── ros2_pubsub_package/
│           ├── ros2_pubsub_package/
│           │   ├── publisher_node.py
│           │   └── subscriber_node.py
│           ├── package.xml
│           └── setup.py
├── aws/
│   ├── start-ec2.sh
│   ├── stop-ec2.sh
│   └── list-instances.sh
├── docker/
│   └── Dockerfile (optional)
├── README.md
```

---

## 🧰 Prerequisites

- AWS Free Tier account with IAM user and EC2 permissions
- AWS CLI configured (`aws configure`)
- SSH access to your EC2 instance
- Docker installed on EC2

---

## ☁️ EC2 Setup (Ubuntu 24.04 Free Tier)

1. Launch EC2 instance:
    ```bash
    aws ec2 run-instances \
      --image-id ami-0fc5d935ebf8bc3bc \
      --instance-type t2.micro \
      --key-name your-key \
      --security-groups your-sg-name \
      --tag-specifications 'ResourceType=instance,Tags=[{Key=Name,Value=ros2-dev}]'
    ```

2. Allocate and associate an **Elastic IP** via AWS Console (or CLI).

3. SSH into instance:
    ```bash
    ssh -i your-key.pem ubuntu@your-elastic-ip
    ```

4. Install Docker:
    ```bash
    sudo apt update && sudo apt install -y docker.io git
    sudo usermod -aG docker $USER
    newgrp docker
    ```

---

## 🐳 Docker & ROS 2 Setup

1. Run ROS 2 Humble container:
    ```bash
    docker run -it --name ros2-dev -v ~/ros2_ws:/ros2_ws osrf/ros:humble
    ```

2. Inside container:
    ```bash
    apt update && apt install -y python3-colcon-common-extensions nano
    cd /ros2_ws
    mkdir -p src && cd src
    ros2 pkg create --build-type ament_python ros2_pubsub_package
    ```

---

## 🧠 Publisher / Subscriber in Python

### `/ros2_ws/src/my_package/my_package/publisher_node.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from Publisher'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

rclpy.init()
rclpy.spin(MinimalPublisher())
rclpy.shutdown()
```

### `/ros2_ws/src/my_package/my_package/subscriber_node.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

rclpy.init()
rclpy.spin(MinimalSubscriber())
rclpy.shutdown()
```

---

## 🔨 Build and Run

1. Inside container:
    ```bash
    cd /ros2_ws
    colcon build
    source install/setup.bash
    ```

2. Open **two terminals** and exec into Docker in each:
    ```bash
    docker exec -it ros2-dev bash
    ```

3. Terminal 1:
    ```bash
    source /ros2_ws/install/setup.bash
    ros2 run my_package publisher_node
    ```

4. Terminal 2:
    ```bash
    source /ros2_ws/install/setup.bash
    ros2 run my_package subscriber_node
    ```

---

## 🧞 AWS Control Scripts (`aws/`)

- `start-ec2.sh` – Starts EC2 instance
- `stop-ec2.sh` – Stops EC2 instance
- `list-instances.sh` – Shows status in table

Example:
```bash
./aws/start-ec2.sh i-xxxxxxxxx us-east-1
```

---

## 📌 Notes

- Don't forget to **stop your EC2 instance** to avoid charges.
- This project can be extended with **Gazebo**, **RViz**, **custom messages**, or **robot simulation**.

---

## 📚 Learn More

- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [AWS CLI Docs](https://docs.aws.amazon.com/cli/)
- [Docker](https://docs.docker.com/engine/install/ubuntu/)

---

## 🛠️ Future Plans

- Add RViz/Gazebo support (remote or NoVNC)
- Add launch files
- Add ROS 2 services and actions
- Automate Docker + ROS 2 startup with `docker-compose`

---

## 🤖 Author

**Vahid Nikougoftar** (+ChatGPT!) — Robotics | AI | Cloud Enthusiast  
[LinkedIn](https://linkedin.com/in/vahidnikougoftar) | [GitHub](https://github.com/vahidnikougoftar)

