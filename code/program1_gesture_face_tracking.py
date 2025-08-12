import cv2
import time
import numpy as np
import face_recognition
import pickle
import mediapipe as mp
import pyzed.sl as sl
from threading import Thread
from roboclaw_lib import Roboclaw
import sys
from collections import deque

# === MediaPipe Pose Setup ===
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# === State ===
use_realsense = True
last_toggle_time = 0
toggle_cooldown = 3
exit_flag = False  # ✅ Global exit flag to stop all threads

# === ZED Setup ===
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER
runtime = sl.RuntimeParameters()
zed_image = sl.Mat()
zed_depth = sl.Mat()

# === Load Face Encodings ===
with open("/home/jetson/jetson_robot/realtime_face_data/known_face.pkl", "rb") as f:
    known_encodings = pickle.load(f)

# === RoboClaw Setup ===
roboclaw = Roboclaw("/dev/ttyACM0", 38400)
roboclaw.comport = "/dev/ttyACM0"
roboclaw.rate = 38400
roboclaw.timeout = 0.1
roboclaw.Open()
address = 128

# === Shared Data ===
current_frame = None
frame_source = "none"
last_frame_source = None

# === RealSense Setup ===
def open_realsense():
    c = cv2.VideoCapture(6)
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return c

# === Detect Raised Arm ===
def detect_arm_up(image, right_arm=True):
    results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    if results.pose_landmarks:
        lm = results.pose_landmarks.landmark
        wrist = lm[mp_pose.PoseLandmark.RIGHT_WRIST if right_arm else mp_pose.PoseLandmark.LEFT_WRIST]
        elbow = lm[mp_pose.PoseLandmark.RIGHT_ELBOW if right_arm else mp_pose.PoseLandmark.LEFT_ELBOW]
        shoulder = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER if right_arm else mp_pose.PoseLandmark.LEFT_SHOULDER]
        if wrist.y < elbow.y < shoulder.y and abs(wrist.x - elbow.x) < 0.05:
            return True
    return False

# === RealSense Thread ===

def realsense_loop():
    global use_realsense, last_toggle_time, current_frame, frame_source, exit_flag

    cap = open_realsense()
    print("✅ RealSense stream ON")

    # === MediaPipe setup for gesture control ===
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose
    mp_hands = mp.solutions.hands
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

    # === State variables ===
    last_command = None
    command_history = deque(maxlen=5)
    speed = 30
    turn_speed = 30

    while use_realsense and not exit_flag:
        ret, frame = cap.read()
        if not ret:
            print("❌ RealSense frame failed.")
            break

        frame_height, frame_width = frame.shape[:2]
        image = cv2.flip(frame.copy(), 1)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # === Pose Detection ===
        pose_results = pose.process(rgb_image)
        pointing_left = False
        pointing_right = False
        forward_pose = False
        backward_pose = False
        stop_pose = False

        if pose_results.pose_landmarks:
            lm = pose_results.pose_landmarks.landmark

            rw = lm[mp_pose.PoseLandmark.RIGHT_WRIST]
            lw = lm[mp_pose.PoseLandmark.LEFT_WRIST]
            rs = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            ls = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
            re = lm[mp_pose.PoseLandmark.RIGHT_ELBOW]
            le = lm[mp_pose.PoseLandmark.LEFT_ELBOW]
            rh = lm[mp_pose.PoseLandmark.RIGHT_HIP]
            lh = lm[mp_pose.PoseLandmark.LEFT_HIP]

            rw_x, rw_y = int(rw.x * frame_width), int(rw.y * frame_height)
            lw_x, lw_y = int(lw.x * frame_width), int(lw.y * frame_height)
            rs_x, rs_y = int(rs.x * frame_width), int(rs.y * frame_height)
            ls_x, ls_y = int(ls.x * frame_width), int(ls.y * frame_height)

            cross_thresh_x = 80
            cross_thresh_y = 80
            y_raise_thresh = 30

            # Crossed arms = BACKWARD
            if abs(rw_x - ls_x) < cross_thresh_x and abs(lw_x - rs_x) < cross_thresh_x:
                if abs(rw_y - ls_y) < cross_thresh_y and abs(lw_y - rs_y) < cross_thresh_y:
                    backward_pose = True

            # Pointing left (right arm across chest)
            if rw_x < ls_x and rw_y < rs_y - y_raise_thresh and rw_x < int(re.x * frame_width):
                pointing_left = True

            # Pointing right (left arm across chest)
            if lw_x > rs_x and lw_y < ls_y - y_raise_thresh and lw_x > int(le.x * frame_width):
                pointing_right = True

            # === Hand Detection ===
            hand_results = hands.process(rgb_image)
            if hand_results.multi_hand_landmarks:
                handLms = hand_results.multi_hand_landmarks[0]

                right_arm_up = rw.y < re.y and re.y < rh.y
                left_arm_up = lw.y < le.y and le.y < lh.y

                if right_arm_up or left_arm_up:
                    mp_drawing.draw_landmarks(image, handLms, mp_hands.HAND_CONNECTIONS)
                    lm = handLms.landmark

                    fingers_up = 0
                    fingers = [mp_hands.HandLandmark.THUMB_TIP,
                               mp_hands.HandLandmark.INDEX_FINGER_TIP,
                               mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                               mp_hands.HandLandmark.RING_FINGER_TIP,
                               mp_hands.HandLandmark.PINKY_TIP]
                    bases = [mp_hands.HandLandmark.THUMB_CMC,
                             mp_hands.HandLandmark.INDEX_FINGER_MCP,
                             mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
                             mp_hands.HandLandmark.RING_FINGER_MCP,
                             mp_hands.HandLandmark.PINKY_MCP]

                    for tip_id, base_id in zip(fingers, bases):
                        if lm[tip_id].y < lm[base_id].y:
                            fingers_up += 1

                    if fingers_up >= 4:
                        stop_pose = True
                    elif fingers_up == 0:
                        forward_pose = True

        # === Decide command ===
        current_command = "STOP"
        if stop_pose:
            current_command = "STOP"
        elif backward_pose:
            current_command = "BACKWARD"
        elif pointing_left:
            current_command = "RIGHT"
        elif pointing_right:
            current_command = "LEFT"
        elif forward_pose:
            current_command = "FORWARD"

        # === Smooth command ===
        command_history.append(current_command)
        smoothed_command = max(set(command_history), key=command_history.count)

        if smoothed_command != last_command:
            last_command = smoothed_command
            print(f"🧠 New command: {smoothed_command}")
            if smoothed_command == "FORWARD":
                roboclaw.ForwardM1(address, speed)
                roboclaw.ForwardM2(address, speed)
            elif smoothed_command == "BACKWARD":
                roboclaw.BackwardM1(address, speed)
                roboclaw.BackwardM2(address, speed)
            elif smoothed_command == "LEFT":
                roboclaw.BackwardM1(address, turn_speed)
                roboclaw.ForwardM2(address, turn_speed)
            elif smoothed_command == "RIGHT":
                roboclaw.ForwardM1(address, turn_speed)
                roboclaw.BackwardM2(address, turn_speed)
            elif smoothed_command == "STOP":
                roboclaw.ForwardM1(address, 0)
                roboclaw.ForwardM2(address, 0)

        # === Switch to ZED if right arm raised ===
        if detect_arm_up(image, right_arm=True) and time.time() - last_toggle_time > toggle_cooldown:
            print("🟡 Switching OFF gesture mode (RealSense → ZED)")
            roboclaw.ForwardM1(address, 0)
            roboclaw.ForwardM2(address, 0)
            last_toggle_time = time.time()
            use_realsense = False
            break

        # === Draw Circle Indicator for Command ===
        color_map = {
            "FORWARD": (0, 140, 255),  # Orange
            "STOP":    (0, 0, 255),    # Red
            "BACKWARD": (0, 255, 0),   # Green
            "LEFT":    (0, 0, 0),      # Black
            "RIGHT":   (255, 0, 0)     # Blue
        }
        indicator_color = color_map.get(smoothed_command, (200, 200, 200))
        cv2.circle(image, center=(100, image.shape[0] - 100), radius=60, color=indicator_color, thickness=-1)

        # === Update shared frame ===
        current_frame = image
        frame_source = "RealSense"

    cap.release()
    roboclaw.ForwardM1(address, 0)
    roboclaw.ForwardM2(address, 0)

# === ZED Thread ===
def zed_loop():
    global use_realsense, last_toggle_time, current_frame, frame_source, exit_flag
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("❌ Failed to open ZED")
        return
    print("✅ ZED stream ON")

    while not use_realsense and not exit_flag:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(zed_image, sl.VIEW.LEFT)
            zed.retrieve_measure(zed_depth, sl.MEASURE.DEPTH)

            frame = np.array(zed_image.get_data(), dtype=np.uint8).copy()
            rgb = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_BGR2RGB)
            small = cv2.resize(rgb, (0, 0), fx=0.5, fy=0.5)

            locations = face_recognition.face_locations(small)
            encodings = face_recognition.face_encodings(small, locations)

            for (top, right, bottom, left), encoding in zip(locations, encodings):
                distances = face_recognition.face_distance(known_encodings, encoding)
                best = np.argmin(distances)
                if distances[best] < 0.55:
                    top *= 2
                    right *= 2
                    bottom *= 2
                    left *= 2
                    cx = (left + right) // 2
                    cy = max(0, (top + bottom) // 2 - 70)

                    depth = zed_depth.get_value(cx, cy)[1]
                    if not np.isnan(depth):
                        # ✅ Draw box and depth label BEFORE flipping the frame
                        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                        cv2.putText(frame, f"{depth:.2f} m", (left, bottom + 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

                        cx_normalized = cx / frame.shape[1]

                        if cx_normalized < 0.4:
                            print("↪️ Face left — turning left.")
                            roboclaw.BackwardM1(address, 30)
                            roboclaw.ForwardM2(address, 30)
                        elif cx_normalized > 0.6:
                            print("↩️ Face right — turning right.")
                            roboclaw.ForwardM1(address, 30)
                            roboclaw.BackwardM2(address, 30)
                        else:
                            print("✅ Face centered — checking depth.")
                            if depth < 0.40:
                                print("⬅️ Too close! Moving backward.")
                                roboclaw.ForwardM1(address, 30)
                                roboclaw.ForwardM2(address, 30)
                            elif depth > 0.5:
                                print("➡️ Too far! Moving forward.")
                                roboclaw.BackwardM1(address, 30)
                                roboclaw.BackwardM2(address, 30)
                            else:
                                print("⏸ In range. Stopping motors.")
                                roboclaw.ForwardM1(address, 0)
                                roboclaw.ForwardM2(address, 0)

            if detect_arm_up(frame, right_arm=False) and time.time() - last_toggle_time > toggle_cooldown:
                print("🟢 Switching ON gesture mode (ZED → RealSense)")
                roboclaw.ForwardM1(address, 0)
                roboclaw.ForwardM2(address, 0)
                last_toggle_time = time.time()
                use_realsense = True
                break

            image = cv2.flip(frame, 1)
            current_frame = image
            frame_source = "ZED"

        time.sleep(0.01)

    zed.close()


# === Main Loop ===
try:
    while True:
        t = Thread(target=realsense_loop if use_realsense else zed_loop)
        t.start()

        while t.is_alive():
            if current_frame is not None:
                if frame_source != last_frame_source:
                    if last_frame_source == "RealSense":
                        cv2.destroyWindow("RealSense Stream")
                    elif last_frame_source == "ZED":
                        cv2.destroyWindow("ZED Stream")
                    last_frame_source = frame_source

                if frame_source == "RealSense":
                    cv2.imshow("RealSense Stream", current_frame)
                elif frame_source == "ZED":
                    cv2.imshow("ZED Stream", current_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("🛑 Exit requested. Stopping motors and all threads.")
                exit_flag = True  # ✅ Stop all threads
                roboclaw.ForwardM1(address, 0)
                roboclaw.ForwardM2(address, 0)
                break

        t.join()
        if exit_flag:
            break

except KeyboardInterrupt:
    print("🔴 Ctrl+C exit.")
    exit_flag = True

# Final cleanup
roboclaw.ForwardM1(address, 0)
roboclaw.ForwardM2(address, 0)
cv2.destroyAllWindows()
sys.exit(0)




