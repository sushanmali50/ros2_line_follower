---

# 🤖 ROS2 Line Follower Robot

An autonomous line-following robot built using **ROS2**, a **Raspberry Pi**, and a **camera**. This project uses **OpenCV** for line detection and leverages ROS2 nodes for real-time image processing and motor control. It was developed as part of a personal robotics project to gain hands-on experience with embedded systems, computer vision, and robotic middleware.

---

## 📸 Demo


---

## 🔧 Tech Stack

- ROS2 (e.g., Humble/Foxy)
- Raspberry Pi 4
- Python 3
- OpenCV
- Camera module (Pi Camera or USB Camera)
- Motor Driver (e.g., TB6612FNG)
- DC Motors + Chassis

---

## 📁 Repository Structure

```
ros2-line-follower/
├── README.md
├── .gitignore
├── images/
│   └── demo.gif
├── launch/
│   └── line_follower.launch.py
├── config/
│   └── params.yaml
├── line_follower/
│   ├── __init__.py
│   ├── line_follower_node.py
│   └── camera_node.py
├── scripts/
│   └── install_dependencies.sh
├── requirements.txt
├── package.xml
└── setup.py
```

---

## ⚙️ Features

- ✅ Real-time line detection using OpenCV
- ✅ Smooth motor control through ROS2 publishers
- ✅ Adjustable thresholds and parameters via config
- ✅ Clean modular design with launch support

---

## 🚀 How to Run

**1. Clone the Repository**

```
git clone https://github.com/sushanmali50/ros2_line_follower.git
cd ros2_line_follower
```

**2. Install Dependencies**

```
bash scripts/install_dependencies.sh
```

Or manually:

```
sudo apt update
sudo apt install python3-opencv
pip install -r requirements.txt
```

**3. Build the Package**

```
colcon build
source install/setup.bash
```

**4. Launch the Line Follower**

```
ros2 launch line_follower line_follower.launch.py
```

---

## ⚙️ Parameters

Editable in `config/params.yaml`:

```
threshold: 150
motor_speed: 0.2
camera_topic: /camera/image_raw
```

---

## 🧠 Future Improvements

- Smooth out jittery movement using PID tuning
- Add obstacle detection (Ultrasonic or LiDAR)
- Deploy object detection with YOLO or TensorFlow Lite
- Implement camera calibration

---

## 💡 Learning Outcomes

- Gained hands-on experience with ROS2 nodes and launch files
- Integrated a real-time camera feed with computer vision
- Controlled a robot using ROS2 on a Raspberry Pi
- Debugged performance bottlenecks on embedded hardware

---

## 📜 License

This project is licensed under the MIT License. See the LICENSE file for details.

---

## 🙋‍♂️ Acknowledgements

- Special thanks to the ROS and OpenCV communities.
- Inspired by The Construct ROS2 Tutorials and countless helpful GitHub projects.

---

## 📫 Contact

Feel free to reach out for questions, collaboration, or feedback!

Your Name – your.email@example.com  
LinkedIn – https://linkedin.com/in/yourprofile  
GitHub – https://github.com/your-username

---
