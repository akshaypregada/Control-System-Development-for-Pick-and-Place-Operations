# Control System Development for Pick-and-Place Operations

## Overview
This project involves the simulation and control of a **UR-3 robotic arm** for pick-and-place operations. The system integrates **PID control, inverse kinematics, motion planning**, and **vision-based object detection** using OpenCV. The control algorithms are tested in **URSim** before real-world deployment.

## Features
- **Advanced Control Algorithms**: Implements **inverse kinematics** and **PID control** for joint-level manipulation.
- **Motion Planning**: Uses **ROS MoveIt** for trajectory optimization and collision avoidance.
- **Vision System**: Object detection through **OpenCV**.
- **Sensor Integration**: Real-time feedback from **vision and tactile sensors**.

## Technologies Used
- **Programming Languages**: Python
- **Tools**: ROS, MoveIt, URSim, OpenCV
- **Hardware**: UR-3 Robotic Arm

## File Structure
```
Control-System-Pick-Place/
│── README.md                  # Project Documentation
│── src/
│   ├── control.py            # PID control and inverse kinematics
│   ├── motion_planning.py    # MoveIt motion planning
│   ├── vision_processing.py  # Object detection
│   ├── sensor_feedback.py    # Sensor integration
│── config/
│   ├── ur3_config.yaml       # UR-3 configuration
│   ├── moveit_config.yaml    # MoveIt parameters
```

## Setup & Usage
1. Install dependencies:
   ```bash
   sudo apt install ros-noetic-moveit
   pip install opencv-python
   ```
2. Run the control system:
   ```bash
   roslaunch ur3_control.launch
   ```

## Contributors
- **Akshay Pregada** 

