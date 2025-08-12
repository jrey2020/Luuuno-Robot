import time
import serial
import signal
import sys
import subprocess
import re
from roboclaw_lib import Roboclaw

# === RoboClaw Setup ===
address = 128
speed = 30
robot = Roboclaw("/dev/ttyACM0", 38400)

if not robot.Open():
    print("❌ RoboClaw not connected.")
    sys.exit(1)
else:
    print("✅ RoboClaw connected")

# === Arduino Serial ===
arduino = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
time.sleep(2)

# === Motor Control ===
def move_forward():
    robot.BackwardM1(address, speed)
    robot.BackwardM2(address, speed)
    print("⬆️ Moving forward...")

def move_backwards():
    robot.ForwardM1(address, speed)
    robot.ForwardM2(address, speed)
    print("⬅️ Reversing...")

def stop():
    robot.ForwardM1(address, 0)
    robot.ForwardM2(address, 0)
    print("🛑 Motors stopped")

def handle_exit(sig, frame):
    stop()
    sys.exit(0)

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

# === Arduino wait ===
def wait_for_turn():
    while True:
        line = arduino.readline().decode().strip()
        if line:
            print(line)
        if "STOP_MOTORS" in line:
            break

# === Tick counter ===
def wait_until_ticks(goal):
    _, start1, _ = robot.ReadEncM1(address)
    _, start2, _ = robot.ReadEncM2(address)
    print(f"🔧 Starting encoders → M1: {start1}, M2: {start2}")

    while True:
        _, enc1, _ = robot.ReadEncM1(address)
        _, enc2, _ = robot.ReadEncM2(address)

        delta1 = abs(enc1 - start1)
        delta2 = abs(enc2 - start2)
        avg_ticks = (delta1 + delta2) // 2

        print(f"📟 Ticks → M1: {delta1}, M2: {delta2}, AVG: {avg_ticks}")
        if avg_ticks >= goal:
            break
        time.sleep(0.05)

    print(f"✅ Tick goal reached: avg={avg_ticks}")

# === STEP 0: LIDAR Approach Wall at 60cm ===
print("🧠 STEP 0: LIDAR approach until 60cm from front wall")

# Start lidar
lidar_proc = subprocess.Popen(
    ["./ultra_simple", "--channel", "--serial", "/dev/ttyUSB0", "1000000"],
    cwd="/home/jetson/rplidar_sdk-master/output/Linux/Release",
    stdout=subprocess.PIPE,
    text=True
)

try:
    lidar_triggered = False
    for line in lidar_proc.stdout:
        match = re.search(r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)', line)
        if not match:
            continue

        angle = float(match.group(1))
        distance = float(match.group(2))

        if distance == 0:
            continue

        # Only monitor FRONT (angle ≈ 0° ±10°)
        if (angle >= 350 or angle <= 10):
            if distance <= 400:
                stop()
                print(f"🧱 Wall detected at {distance:.0f} mm → Stopping.")
                lidar_triggered = True
                break
            else:
                if not lidar_triggered:
                    move_forward()

except KeyboardInterrupt:
    print("🛑 Ctrl+C pressed during LIDAR front scan.")
    stop()

lidar_proc.terminate()
print("⏳ Pausing 2 seconds before first turn...")
time.sleep(2)

# === STEP 1: First 90° Turn ===
print("🌀 First 90° turn")
arduino.write(b"TURN_LEFT\n")
robot.BackwardM1(address, speed)
robot.ForwardM2(address, speed)
wait_for_turn()
stop()
time.sleep(2)

# === STEP 2: Drive forward 4000 ticks
print("🚗 Driving forward 4000 ticks")
robot.ResetEncoders(address)
time.sleep(0.2)
robot.BackwardM1(address, speed)
robot.BackwardM2(address, speed)
wait_until_ticks(8000)
stop()
time.sleep(2)

# === STEP 3: Simulated 180° turn (two 90° turns)
for i in range(2):
    print(f"🔁 Simulated 180° turn → Part {i+1}/2")
    arduino.write(b"TURN_LEFT\n")
    robot.BackwardM1(address, speed)
    robot.ForwardM2(address, speed)
    wait_for_turn()
    stop()
    time.sleep(1)

time.sleep(2)

# === STEP 4: Drive forward another 4000 ticks
print("🚗 Driving forward another 4000 ticks")
robot.ResetEncoders(address)
time.sleep(0.2)
robot.BackwardM1(address, speed)
robot.BackwardM2(address, speed)
wait_until_ticks(8000)
stop()
time.sleep(2)

# === STEP 5: Final 90° turn
arduino.write(b"TURN_LEFT\n")
print("🌀 Final 90° turn")
robot.BackwardM1(address, speed)
robot.ForwardM2(address, speed)
wait_for_turn()
stop()

# === STEP 6: Reverse into parking until 20cm from back wall
print("🅿️ Activating LIDAR for reverse parking...")

# Start lidar
lidar_proc = subprocess.Popen(
    ["./ultra_simple", "--channel", "--serial", "/dev/ttyUSB0", "1000000"],
    cwd="/home/jetson/rplidar_sdk-master/output/Linux/Release",
    stdout=subprocess.PIPE,
    text=True
)

try:
    for line in lidar_proc.stdout:
        match = re.search(r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)', line)
        if not match:
            continue

        angle = float(match.group(1))
        distance = float(match.group(2))
        if distance == 0:
            continue

        # Only monitor BACK (angle ≈ 180° ±10°)
        if 170 <= angle <= 190:
            if distance <= 150:
                stop()
                print(f"✅ Parked! Object at {distance:.0f} mm behind.")
                break
            else:
                move_backwards()

except KeyboardInterrupt:
    print("🛑 Ctrl+C pressed.")
    stop()

lidar_proc.terminate()
print("🏁 PROGRAM COMPLETE — FULLY PARKED")
