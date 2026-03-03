import serial
import cv2
import mediapipe as mp
import time
from .utils import clamp, map_range

class RobotController:
    def __init__(self, port='COM5', baudrate=115200, debug=False):
        self.debug = debug
        self.ser = None
        self.HOME_POSITION = [90, 90, 90, 60, 90]
        self.current_angles = self.HOME_POSITION.copy()
        self.target_angles = self.HOME_POSITION.copy()
        self.prev_angles = self.HOME_POSITION.copy()
        self.velocities = [0.0] * 5
        self.MAX_SPEED = 10
        self.ACCELERATION = 0.2
        self.SMOOTHING_ALPHA = 0.3
        self.NO_HAND_TIMEOUT = 0.5
        self.dead_zone = 1
        self.last_send_time = time.time()
        self.last_hand_detected_time = time.time()
        self.hand_present = False
        self.SMOOTHING_GRIP = 0.6
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        if not self.debug:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
            except Exception as e:
                self.debug = True

    def _calculate_extension(self, landmarks, tip_id, mcp_id, pip_id):
        tip = landmarks.landmark[tip_id]
        pip = landmarks.landmark[pip_id]
        mcp = landmarks.landmark[mcp_id]
        wrist = landmarks.landmark[0]
        tip_to_pip = ((tip.x - pip.x)**2 + (tip.y - pip.y)**2 + (tip.z - pip.z)**2)**0.5
        mcp_to_wrist = ((mcp.x - wrist.x)**2 + (mcp.y - wrist.y)**2 + (mcp.z - wrist.z)**2)**0.5
        return clamp(((tip_to_pip / mcp_to_wrist) - 0.1) / 0.3, 0, 1)

    def get_angles_from_landmarks(self, landmarks):
        wrist = landmarks.landmark[0]
        idx_mcp = landmarks.landmark[5]
        idx_tip = landmarks.landmark[8]
        thumb_tip = landmarks.landmark[4]
        mid_mcp = landmarks.landmark[9]
        palm_x = (wrist.x + idx_mcp.x + mid_mcp.x) / 3
        hand_x = clamp(palm_x - 0.5, -0.2, 0.2)
        base = map_range(hand_x, -0.2, 0.2, 0, 180)
        shoulder = map_range(clamp(wrist.y, 0.3, 0.9), 0.3, 0.9, 180, 0)
        palm_size = ((wrist.x - idx_mcp.x)**2 + (wrist.y - idx_mcp.y)**2)**0.5
        elbow = map_range(clamp(palm_size, 0.1, 0.3), 0.1, 0.3, 10, 180)
        idx_ext = self._calculate_extension(landmarks, 8, 5, 6)
        mid_ext = self._calculate_extension(landmarks, 12, 9, 10)
        combined_ext = clamp((idx_ext + mid_ext) / 2, 0.3, 0.8)
        wrist_rot = map_range(combined_ext, 0.3, 0.8, 10, 120)
        pinch_dist = ((thumb_tip.x - idx_tip.x)**2 + (thumb_tip.y - idx_tip.y)**2)**0.5
        gripper = map_range(clamp(pinch_dist, 0.03, 0.10), 0.03, 0.10, 0, 180)
        return [base, shoulder, elbow, wrist_rot, gripper], combined_ext

    def update(self, frame):
        current_time = time.time()
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        combined_ext = 0.5
        if results.multi_hand_landmarks:
            self.hand_present = True
            self.last_hand_detected_time = current_time
            raw_target, combined_ext = self.get_angles_from_landmarks(results.multi_hand_landmarks[0])
            for i in range(5):
                if(i==4):
                    self.SMOOTHING_ALPHA =self.SMOOTHING_GRIP = 0.6

                self.target_angles[i] = (self.SMOOTHING_ALPHA * raw_target[i] + (1 - self.SMOOTHING_ALPHA) * self.target_angles[i])
                    
        else:
            if current_time - self.last_hand_detected_time > self.NO_HAND_TIMEOUT:
                self.hand_present = False
                for i in range(5):
                    self.target_angles[i] += (self.HOME_POSITION[i] - self.target_angles[i]) * 0.05
        for i in range(5):
            error = self.target_angles[i] - self.current_angles[i]
            if abs(error) > 0.1:
                if i == 4:
                    self.current_angles[i] = self.target_angles[i]
                    self.velocities[i] = 0
                else:
                    desired_vel = error * 0.1
                    vel_change = clamp(desired_vel - self.velocities[i], -self.ACCELERATION, self.ACCELERATION)
                    self.velocities[i] = clamp(self.velocities[i] + vel_change, -self.MAX_SPEED, self.MAX_SPEED)
                    self.current_angles[i] += self.velocities[i]
        return results, combined_ext

    def send_to_robot(self):
        movement = sum(abs(self.current_angles[i] - self.prev_angles[i]) for i in range(5))
        if movement > self.dead_zone or not self.hand_present:
            if not self.debug and self.ser:
                self.ser.write(bytearray([int(a) for a in self.current_angles]))
            self.prev_angles = self.current_angles.copy()
            self.last_send_time = time.time()
    
    def close(self):
        """Safe shutdown of serial and vision resources."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
        if hasattr(self, 'hands'):
            self.hands.close()
            print("Mediapipe resources released.")