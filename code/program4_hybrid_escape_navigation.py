import subprocess
import re
import time
import serial
import sys
from roboclaw_lib import Roboclaw
import pyzed.sl as sl
import cv2
import numpy as np

# === Constants ===
THRESHOLD_CM = 40
THRESHOLD_MM = THRESHOLD_CM * 10
EXIT_CM = 170
EXIT_MM = EXIT_CM * 10
ENCODER_THRESHOLD_TICKS = 1000

# === Setup RoboClaw and Arduino ===
address = 128
speed = 38

robot = Roboclaw("/dev/ttyACM0", 38400)
if not robot.Open():
    print("❌ RoboClaw not connected.")
    sys.exit(1)

arduino = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
time.sleep(2)

# === Setup ZED SLAM ===
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.coordinate_units = sl.UNIT.MILLIMETER

status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(f"❌ Failed to open ZED Camera: {status}")
    sys.exit(1)

tracking_params = sl.PositionalTrackingParameters()
status = zed.enable_positional_tracking(tracking_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(f"❌ Failed to enable ZED SLAM: {status}")
    zed.close()
    sys.exit(1)

runtime_params = sl.RuntimeParameters()
pose = sl.Pose()
last_zed_z = None
step_counter = 0
exit_sign_detected = False

# === Load ORB EXIT template ===
template = cv2.imread("/home/jetson/exit.jpg", cv2.IMREAD_GRAYSCALE)
orb = cv2.ORB_create(nfeatures=1000)
template_kp, template_des = orb.detectAndCompute(template, None)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# === Track last encoder position ===
last_enc1 = None
last_enc2 = None
last_direction = None

def stop():
    robot.ForwardM1(address, 0)
    robot.ForwardM2(address, 0)
    print("🛑 Motors stopped")

def read_encoders():
    enc1 = robot.ReadEncM1(address)[1]
    enc2 = robot.ReadEncM2(address)[1]
    return enc1, enc2

def encoder_delta(e1a, e2a, e1b, e2b):
    return abs(e1b - e1a) + abs(e2b - e2a)

def send_turn_cmd(cmd):
    arduino.write((cmd + "\n").encode())
    print(f"🧽 Sent to Arduino: {cmd}")
    while True:
        line = arduino.readline().decode().strip()
        if "TURN_LEFT_NOW" in line:
            robot.BackwardM1(address, speed)
            robot.ForwardM2(address, speed)
        elif "TURN_RIGHT_NOW" in line:
            robot.ForwardM1(address, speed)
            robot.BackwardM2(address, speed)
        elif "TURN_180_NOW" in line:
            robot.BackwardM1(address, speed)
            robot.ForwardM2(address, speed)
        elif "STOP_MOTORS" in line:
            break
    stop()
    time.sleep(0.3)

def get_lidar_distances():
    distances = {"Front": None, "Left": None, "Right": None, "Back": None}
    print("🔄 Reading LIDAR...")
    lidar_proc = subprocess.Popen(
        ["./ultra_simple", "--channel", "--serial", "/dev/ttyUSB0", "1000000"],
        cwd="/home/jetson/rplidar_sdk-master/output/Linux/Release",
        stdout=subprocess.PIPE,
        text=True
    )
    try:
        start_time = time.time()
        while time.time() - start_time < 2.0:
            line = lidar_proc.stdout.readline()
            match = re.search(r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)', line)
            if not match:
                continue
            angle = float(match.group(1))
            distance = float(match.group(2))
            if distance == 0:
                continue
            if (angle >= 350 or angle <= 10) and distances["Front"] is None:
                distances["Front"] = distance
            elif 80 <= angle <= 100 and distances["Right"] is None:
                distances["Right"] = distance
            elif 260 <= angle <= 280 and distances["Left"] is None:
                distances["Left"] = distance
            elif 170 <= angle <= 190 and distances["Back"] is None:
                distances["Back"] = distance
    finally:
        lidar_proc.terminate()
    return distances

def drive_forward_until_wall():
    print("🕒 Waiting 2 seconds...")
    time.sleep(2)
    print("🚗 Moving forward. Watching front LIDAR (345°–15°)...")
    lidar_proc = subprocess.Popen(
        ["./ultra_simple", "--channel", "--serial", "/dev/ttyUSB0", "1000000"],
        cwd="/home/jetson/rplidar_sdk-master/output/Linux/Release",
        stdout=subprocess.PIPE,
        text=True
    )
    robot.BackwardM1(address, speed)
    robot.BackwardM2(address, speed)
    try:
        while True:
            line = lidar_proc.stdout.readline()
            match = re.search(r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)', line)
            if not match:
                continue
            angle = float(match.group(1))
            distance = float(match.group(2))
            if distance == 0:
                continue
            if (angle >= 345 or angle <= 15) and distance <= THRESHOLD_MM:
                print(f"🧱 Wall detected at {distance:.0f} mm — stopping.")
                break
    finally:
        lidar_proc.terminate()
        stop()
        print("🌝 Reached wall — reassessing...")

# === MAIN LOOP ===
image = sl.Mat()
try:
    while True:
        # Check for EXIT sign using ORB
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            kp, des = orb.detectAndCompute(gray, None)
            if des is not None and template_des is not None:
                matches = bf.match(template_des, des)
                matches = sorted(matches, key=lambda x: x.distance)
                good = matches[:30]

                if len(good) >= 10:
                    src_pts = np.float32([template_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    if M is not None and mask is not None:
                        inliers = mask.ravel().tolist().count(1)
                        if inliers >= 10:
                            print("✅ EXIT SIGN DETECTED")
                            exit_sign_detected = True

        distances = get_lidar_distances()
        print("\n📍 LIDAR Readings:")
        for dir in ["Front", "Left", "Right", "Back"]:
            val = distances[dir]
            print(f"{dir}: {val:.0f} mm" if val else f"{dir}: ❌ No reading")

        # ✅ Exit Detection at Left or Right ≥ 200 cm
        if (distances["Left"] and distances["Left"] >= 2800) or (distances["Right"] and distances["Right"] >= 2800):
            print("🟥 EXIT DETECTED — Preparing to park...")

            # Turn 180°
            send_turn_cmd("TURN_180")

            # Back up until wall is 20cm away
            print("🔄 Reversing until back wall is 20cm away...")
            lidar_proc = subprocess.Popen(
                ["./ultra_simple", "--channel", "--serial", "/dev/ttyUSB0", "1000000"],
                cwd="/home/jetson/rplidar_sdk-master/output/Linux/Release",
                stdout=subprocess.PIPE,
                text=True
            )
            robot.ForwardM1(address, speed)
            robot.ForwardM2(address, speed)
            try:
                while True:
                    line = lidar_proc.stdout.readline()
                    match = re.search(r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)', line)
                    if not match:
                        continue
                    angle = float(match.group(1))
                    distance = float(match.group(2))
                    if 170 <= angle <= 190 and distance <= 200:
                        print(f"🧱 Final wall reached at {distance:.0f} mm — parking complete.")
                        break
            finally:
                lidar_proc.terminate()
                stop()
                zed.close()
                cv2.destroyAllWindows()
                sys.exit(0)

        valid_paths = {
            d: distances[d] for d in ["Front", "Left", "Right", "Back"]
            if distances[d] is not None and distances[d] >= THRESHOLD_MM
        }

        if not exit_sign_detected and last_direction in valid_paths:
            print(f"❌ Preventing loop — removing {last_direction} from options")
            del valid_paths[last_direction]

        # ✅ MODIFIED BLOCK — block BACK after Step 2
        if step_counter >= 3 and "Back" in valid_paths:
            print("🚫 BLOCKING BACK — already passed Step 2")
            del valid_paths["Back"]

        if exit_sign_detected and "Back" in valid_paths:
            print("🚫 Blocking BACK due to EXIT SIGN")
            del valid_paths["Back"]

        if not valid_paths:
            print("❌ All paths lead to backtracked areas. Stopping.")
            stop()
            break

        best_dir = max(valid_paths, key=valid_paths.get)
        best_dist = valid_paths[best_dir]
        print(f"\n✅ Decision: GO {best_dir} ({best_dist:.0f} mm)")

        enc1_before, enc2_before = read_encoders()

        if best_dir == "Left":
            send_turn_cmd("TURN_LEFT")
        elif best_dir == "Right":
            send_turn_cmd("TURN_RIGHT")
        elif best_dir == "Back":
            send_turn_cmd("TURN_180")
        else:
            print("🚗 Going forward — no turn")

        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            last_zed_z = pose.get_translation().get()[2]

        drive_forward_until_wall()

        enc1_after, enc2_after = read_encoders()
        delta = encoder_delta(enc1_before, enc2_before, enc1_after, enc2_after)
        print(f"📿 Encoder delta after move: {delta}")

        if delta > ENCODER_THRESHOLD_TICKS:
            last_direction = {
                "Front": "Back",
                "Back": "Front",
                "Left": "Right",
                "Right": "Left"
            }[best_dir]
            step_counter += 1
        else:
            print("⚠️ Movement too small — possibly stuck. Not updating last_direction.")

except KeyboardInterrupt:
    print("\n🛑 Emergency stop triggered (Ctrl+C)!")
    stop()
    zed.close()
    cv2.destroyAllWindows()
    sys.exit(0)
