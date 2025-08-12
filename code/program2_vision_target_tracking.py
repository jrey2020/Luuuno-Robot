import cv2
import numpy as np
import pyzed.sl as sl
from roboclaw_lib import Roboclaw

# === ZED Camera Setup ===
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER
runtime = sl.RuntimeParameters()
zed_image = sl.Mat()
zed_depth = sl.Mat()

# === Roboclaw Setup ===
roboclaw = Roboclaw("/dev/ttyACM0", 38400)
roboclaw.Open()
address = 128

# === ArUco Setup ===
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# === Open ZED Camera ===
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("❌ Failed to open ZED")
    exit(1)
print("✅ ZED is running")

while True:
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(zed_image, sl.VIEW.LEFT)
        zed.retrieve_measure(zed_depth, sl.MEASURE.DEPTH)

        frame = np.array(zed_image.get_data(), dtype=np.uint8).copy()
        gray = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and 0 in ids:
            index = list(ids.flatten()).index(0)
            marker_corners = corners[index][0]

            # Compute center of the marker
            cx = int(np.mean(marker_corners[:, 0]))
            cy = int(np.mean(marker_corners[:, 1]))

            # Get distance from ZED depth map
            depth = zed_depth.get_value(cx, cy)[1]

            if not np.isnan(depth):
                print(f"✅ Marker ID 0 detected at {depth:.2f} meters")

                # Draw bounding box and depth label
                cv2.polylines(frame, [marker_corners.astype(int)], True, (0, 255, 0), 2)
                cv2.putText(frame, f"{depth:.2f} m", (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

                # Normalize horizontal position
                cx_normalized = cx / frame.shape[1]

                if cx_normalized < 0.4:
                    print("↪️ Marker left — turning left.")
                    roboclaw.BackwardM1(address, 20)
                    roboclaw.ForwardM2(address, 20)
                elif cx_normalized > 0.6:
                    print("↩️ Marker right — turning right.")
                    roboclaw.ForwardM1(address, 20)
                    roboclaw.BackwardM2(address, 20)
                else:
                    print("✅ Marker centered — checking depth.")
                    if depth < 0.40:
                        print("⬅️ Too close! Moving backward.")
                        roboclaw.ForwardM1(address, 20)
                        roboclaw.ForwardM2(address, 20)
                    elif depth > 0.50:
                        print("➡️ Too far! Moving forward.")
                        roboclaw.BackwardM1(address, 20)
                        roboclaw.BackwardM2(address, 20)
                    else:
                        print("⏸ In range. Stopping motors.")
                        roboclaw.ForwardM1(address, 0)
                        roboclaw.ForwardM2(address, 0)
        else:
            print("❌ Marker not detected. Stopping.")
            roboclaw.ForwardM1(address, 0)
            roboclaw.ForwardM2(address, 0)

        # Show stream
        cv2.imshow("ZED ArUco Tracker", frame[:, :, :3])

        # Quit on Q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("❎ Quitting program.")
            break

# === Safe Shutdown ===
roboclaw.ForwardM1(address, 0)
roboclaw.ForwardM2(address, 0)
zed.close()
cv2.destroyAllWindows()
