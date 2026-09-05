print("SCARA Autopilot: Auto-Homing + Visual Overlay")
import cv2
import numpy as np
import serial
import time

esp32_port = 'COM4' 
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

# --- CALIBRATION CONSTANTS ---
ROBOT_BASE_X_PIXEL = 243
ROBOT_BASE_Y_PIXEL = 320
MM_PER_PIXEL = 0.875

# --- COLOR PROFILES ---
lower_green = np.array([40, 100, 100]) # For end-effector sticker
upper_green = np.array([80, 255, 255])

lower_yellow = np.array([20, 100, 100]) # For target objects
upper_yellow = np.array([40, 255, 255])

# --- PHASE 1: VISUAL HOMING ---
print("Locating Green end-effector for calibration...")
homed = False

while not homed:
    success, frame = cap.read()
    if not success: continue
    
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        biggest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest_contour) > 200:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                real_x = round((ROBOT_BASE_X_PIXEL - cx) * MM_PER_PIXEL, 1)
                real_y = round((cy - ROBOT_BASE_Y_PIXEL) * MM_PER_PIXEL, 1)
                
                command = f"VHOME {real_x},{real_y}\n"
                esp32.write(command.encode('utf-8'))
                print(f"End-effector found at X:{real_x}, Y:{real_y}. Homing...")
                
                homed = True 
                time.sleep(1) # Allow ESP32 to calculate and reply

    cv2.imshow("SCARA Autopilot", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break


# --- PHASE 2: AUTONOMOUS TRACKING ---
robot_ready = False
print("Waiting for 'READY' signal from robot...")

while True:
    # 1. READ INCOMING SERIAL
    if esp32.in_waiting > 0:
        incoming = esp32.readline().decode('utf-8').strip()
        if "READY" in incoming:
            print("[ROBOT READY] Searching for yellow objects...")
            robot_ready = True
            time.sleep(0.5) 

    # 2. PROCESS VIDEO
    success, frame = cap.read()
    if not success: break
        
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # --- DRAW VISUAL GRID ---
    cv2.line(frame, (0, ROBOT_BASE_Y_PIXEL), (640, ROBOT_BASE_Y_PIXEL), (255, 255, 255), 1)
    cv2.line(frame, (ROBOT_BASE_X_PIXEL, 0), (ROBOT_BASE_X_PIXEL, 480), (255, 255, 255), 1)
    cv2.circle(frame, (ROBOT_BASE_X_PIXEL, ROBOT_BASE_Y_PIXEL), 6, (255, 0, 0), -1)
    cv2.putText(frame, "Base", (ROBOT_BASE_X_PIXEL + 10, ROBOT_BASE_Y_PIXEL - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    step_px = int(100 / MM_PER_PIXEL)

    for i in range(1, 4):
        # X-Axis Ticks
        tick_pos = ROBOT_BASE_X_PIXEL - (i * step_px)
        if 0 <= tick_pos <= 640:
            cv2.line(frame, (tick_pos, ROBOT_BASE_Y_PIXEL - 5), (tick_pos, ROBOT_BASE_Y_PIXEL + 5), (200, 200, 200), 2)
            cv2.putText(frame, f"{i*100}", (tick_pos - 15, ROBOT_BASE_Y_PIXEL + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        tick_neg = ROBOT_BASE_X_PIXEL + (i * step_px)
        if 0 <= tick_neg <= 640:
            cv2.line(frame, (tick_neg, ROBOT_BASE_Y_PIXEL - 5), (tick_neg, ROBOT_BASE_Y_PIXEL + 5), (200, 200, 200), 2)
            cv2.putText(frame, f"{-i*100}", (tick_neg - 15, ROBOT_BASE_Y_PIXEL + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Y-Axis Ticks
        tick_pos_y = ROBOT_BASE_Y_PIXEL + (i * step_px)
        if 0 <= tick_pos_y <= 480:
            cv2.line(frame, (ROBOT_BASE_X_PIXEL - 5, tick_pos_y), (ROBOT_BASE_X_PIXEL + 5, tick_pos_y), (200, 200, 200), 2)
            cv2.putText(frame, f"{i*100}", (ROBOT_BASE_X_PIXEL + 10, tick_pos_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        tick_neg_y = ROBOT_BASE_Y_PIXEL - (i * step_px)
        if 0 <= tick_neg_y <= 480:
            cv2.line(frame, (ROBOT_BASE_X_PIXEL - 5, tick_neg_y), (ROBOT_BASE_X_PIXEL + 5, tick_neg_y), (200, 200, 200), 2)
            cv2.putText(frame, f"{-i*100}", (ROBOT_BASE_X_PIXEL + 10, tick_neg_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # --- DETECT YELLOW TARGETS ---
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
                
                print(f"[ACTION] Sending {command.strip()} and waiting...")
                robot_ready = False 

    # --- DRAW TARGET HIGHLIGHTS ---
    if len(contours) > 0:
        biggest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest_contour) > 500:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                display_x = round((ROBOT_BASE_X_PIXEL - cx) * MM_PER_PIXEL, 1)
                display_y = round((cy - ROBOT_BASE_Y_PIXEL) * MM_PER_PIXEL, 1)

                cv2.drawContours(frame, [biggest_contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                
                text = f"Target: X:{display_x} Y:{display_y}"
                cv2.rectangle(frame, (cx - 25, cy - 35), (cx + 180, cy - 15), (0, 0, 0), -1)
                cv2.putText(frame, text, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.imshow("SCARA Autopilot", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
esp32.close()
cv2.destroyAllWindows()