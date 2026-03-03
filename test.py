import cv2
import mediapipe as mp
from robot_visionaries import RobotController

# 1. Initialize the Controller
# Change 'COM5' to your actual port. 
# Set debug=False to actually send data to the Arduino.
robot = RobotController(port='COM5', baudrate=115200, debug=False)

# Setup Mediapipe drawing for visual feedback
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)

print("--- Robot Visionaries: Hardware Test Mode ---")
print("Press 'ESC' to emergency stop and close.")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue

    # Flip the image horizontally for a mirror-view feel
    frame = cv2.flip(frame, 1)

    # 2. Process the frame through the library
    # This updates current_angles, target_angles, and calculates extensions
    results, combined_ext = robot.update(frame)

    # 3. Send the smoothed angles to the Arduino
    robot.send_to_robot()

    # --- Visual Feedback (UI) ---
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw the hand skeleton on the screen
            mp_drawing.draw_landmarks(
                frame, 
                hand_landmarks, 
                mp_hands.HAND_CONNECTIONS
            )

        # Display the current Servo Angles on the screen
        y_pos = 30
        for i, angle in enumerate(robot.current_angles):
            cv2.putText(frame, f"Servo {i}: {int(angle)} deg", (10, y_pos), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_pos += 30
        
        # Display Wrist Extension Status
        color = (0, 255, 0) if combined_ext > 0.6 else (0, 0, 255)
        cv2.putText(frame, f"Wrist Ext: {combined_ext:.2f}", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    else:
        cv2.putText(frame, "NO HAND DETECTED - HOMING...", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Show the frame
    cv2.imshow('Robot Visionaries - Real-Time Control', frame)

    # Exit on ESC
    if cv2.waitKey(5) & 0xFF == 27:
        break

# 4. Cleanup
robot.close()
cap.release()
cv2.destroyAllWindows()
