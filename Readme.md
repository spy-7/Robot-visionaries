# Robot Visionaries 

"Robot Visionaries" is a powerful, lightweight Python library designed to bridge the gap between computer vision and physical robotics. It allows you to control a 4-DOF (Degree of Freedom) Robotic Arm in real-time using just a standard webcam and hand gestures.

Powered by "Google Mediapipe", this library translates complex hand landmarks into smooth, synchronized servo movements.

---

## Key Features

1."Real-Time Tracking": Ultra-low latency hand tracking using Mediapipe.
2. "Intuitive Gesture Control":
     "Base": Hand horizontal position (X-axis).
     "Shoulder": Wrist vertical position (Y-axis).
     "Elbow": Palm size (distance from camera/Z-axis).
     "Wrist Rotation": Combined extension of Index and Middle fingers.
    "Gripper": Precision pinch detection (Thumb to Index).
2."Smooth Motion": Integrated acceleration and speed control to prevent jerky servo movements.
3. "Auto-Homing": Automatically returns the robot to a safe "Home" position when no hand is detected.
*4."Debug Mode": Test your vision logic without needing the hardware connected.

---

## Installation

Install the library directly from PyPI:

```bash
pip install robot-visionaries