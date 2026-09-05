print("SCARA Autopilot: Vision + Handshake + Visual Overlay")
import cv2
import numpy as np
import serial
import time

esp32_port = 'COM4' # Update to your port
try:
    esp32 = serial.Serial(esp32_port, 115200, timeout=0.1)
    print(f"Connected to ESP32 on {esp32_port}")
except:
    print(f"Error: Could not open {esp32_port}. Is Arduino Serial Monitor closed?")
    exit()

camera_url = "http://192.168.0.181:8080/video"
cap = cv2.VideoCapture(camera_url)

if not cap.isOpened():
    print("Error connecting to camera.")
    exit()

# --- AUTONOMY VARIABLES ---
robot_ready = False
print("Waiting for 'READY' signal from robot...")

# --- CALIBRATION CONSTANTS ---
# Moved here so they can be used for drawing the persistent grid
ROBOT_BASE_X_PIXEL = 243
ROBOT_BASE_Y_PIXEL = 320
MM_PER_PIXEL = 0.875

while True:
    # 1. CHECK MESSAGES FROM ROBOT
    if esp32.in_waiting > 0:
        incoming = esp32.readline().decode('utf-8').strip()
        if "READY" in incoming:
            print("[ROBOT READY] Searching for objects...")
            robot_ready = True
            time.sleep(1) # Give the camera a second to stabilize after arm moves

    # 2. PROCESS VIDEO
    success, frame = cap.read()
    if not success: break
        
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # =================================================================
    # NEW: VISUAL AXIS & COORDINATE GRID OVERLAY
    # =================================================================
    
    # Draw Main X and Y Axes (White lines)
    cv2.line(frame, (0, ROBOT_BASE_Y_PIXEL), (640, ROBOT_BASE_Y_PIXEL), (255, 255, 255), 1)
    cv2.line(frame, (ROBOT_BASE_X_PIXEL, 0), (ROBOT_BASE_X_PIXEL, 480), (255, 255, 255), 1)
    
    # Mark the Origin / Robot Base
    cv2.circle(frame, (ROBOT_BASE_X_PIXEL, ROBOT_BASE_Y_PIXEL), 6, (255, 0, 0), -1)
    cv2.putText(frame, "Base (0,0)", (ROBOT_BASE_X_PIXEL + 10, ROBOT_BASE_Y_PIXEL - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Draw 100mm tick marks based on your specific Inverse Kinematics math
    step_px = int(100 / MM_PER_PIXEL)

    # X-Axis Ticks (100mm, 200mm, etc.)
    for i in range(1, 4):
        # Positive X (Moves left on screen based on your formula: ROBOT_BASE_X - cx)
        tick_pos = ROBOT_BASE_X_PIXEL - (i * step_px)
        if 0 <= tick_pos <= 640:
            cv2.line(frame, (tick_pos, ROBOT_BASE_Y_PIXEL - 5), (tick_pos, ROBOT_BASE_Y_PIXEL + 5), (200, 200, 200), 2)
            cv2.putText(frame, f"{i*100}", (tick_pos - 15, ROBOT_BASE_Y_PIXEL + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Negative X (Moves right on screen)
        tick_neg = ROBOT_BASE_X_PIXEL + (i * step_px)
        if 0 <= tick_neg <= 640:
            cv2.line(frame, (tick_neg, ROBOT_BASE_Y_PIXEL - 5), (tick_neg, ROBOT_BASE_Y_PIXEL + 5), (200, 200, 200), 2)
            cv2.putText(frame, f"{-i*100}", (tick_neg - 15, ROBOT_BASE_Y_PIXEL + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # Y-Axis Ticks (100mm, 200mm, etc.)
    for i in range(1, 4):
        # Positive Y (Moves down on screen based on your formula: cy - ROBOT_BASE_Y)
        tick_pos = ROBOT_BASE_Y_PIXEL + (i * step_px)
        if 0 <= tick_pos <= 480:
            cv2.line(frame, (ROBOT_BASE_X_PIXEL - 5, tick_pos), (ROBOT_BASE_X_PIXEL + 5, tick_pos), (200, 200, 200), 2)
            cv2.putText(frame, f"{i*100}", (ROBOT_BASE_X_PIXEL + 10, tick_pos + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Negative Y (Moves up on screen)
        tick_neg = ROBOT_BASE_Y_PIXEL - (i * step_px)
        if 0 <= tick_neg <= 480:
            cv2.line(frame, (ROBOT_BASE_X_PIXEL - 5, tick_neg), (ROBOT_BASE_X_PIXEL + 5, tick_neg), (200, 200, 200), 2)
            cv2.putText(frame, f"{-i*100}", (ROBOT_BASE_X_PIXEL + 10, tick_neg + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # =================================================================

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 3. IF ROBOT IS READY AND OBJECT FOUND -> SEND COMMAND
    if robot_ready and len(contours) > 0:
        biggest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest_contour) > 500:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                real_x = round((ROBOT_BASE_X_PIXEL - cx) * MM_PER_PIXEL, 1)
                real_y = round((cy - ROBOT_BASE_Y_PIXEL) * MM_PER_PIXEL, 1)
                
                command = f"VPICK {real_x},{real_y}\n"
                esp32.write(command.encode('utf-8'))
                
                print(f"[ACTION] Sending {command.strip()} and waiting for completion...")
                robot_ready = False 

    # Draw graphics for the detected object
    if len(contours) > 0:
        biggest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest_contour) > 500:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate real_x and real_y again just for the continuous visual display
                display_x = round((ROBOT_BASE_X_PIXEL - cx) * MM_PER_PIXEL, 1)
                display_y = round((cy - ROBOT_BASE_Y_PIXEL) * MM_PER_PIXEL, 1)

                cv2.drawContours(frame, [biggest_contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                
                # Add a distinct background box for the text so it stands out against the grid
                text = f"Target: X:{display_x} Y:{display_y}"
                cv2.rectangle(frame, (cx - 25, cy - 35), (cx + 180, cy - 15), (0, 0, 0), -1)
                cv2.putText(frame, text, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.imshow("SCARA Autopilot", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
esp32.close()
cv2.destroyAllWindows()