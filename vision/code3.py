print("SCARA vision Code: Object Tracking + Serial")
import cv2
import numpy as np
import serial # <-- USB দিয়ে ডাটা পাঠানোর জন্য

# =================================================================
# NEW: ESP32 SERIAL CONNECTION
# আপনার ESP32 এর পোর্ট নাম্বারটি এখানে দিন (যেমন: 'COM3', 'COM4')
esp32_port = 'COM4' 
try:
    esp32 = serial.Serial(esp32_port, 115200, timeout=1)
    print(f"Connected to ESP32 on {esp32_port}")
except:
    print(f"Error: Could not open {esp32_port}. Is Arduino Serial Monitor closed?")
    exit()
# =================================================================

camera_url = "http://192.168.0.181:8080/video"
print("Connecting to camera...")
cap = cv2.VideoCapture(camera_url)

if not cap.isOpened():
    print("Error: Could not connect to the camera.")
    exit()

print("Camera connected! Press 'q' to quit, press 'p' to PICK object.")

# ভ্যারিয়েবলগুলো ইনিশিয়ালাইজ করা
real_x = 0.0
real_y = 0.0
object_found = False

while True:
    success, frame = cap.read()
    if not success:
        break
        
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    object_found = False # প্রতি ফ্রেমে শুরুতে False ধরে নিব

    if len(contours) > 0:
        biggest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest_contour) > 500:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # আপনার ক্যালিব্রেশন মান
                ROBOT_BASE_X_PIXEL = 243
                ROBOT_BASE_Y_PIXEL = 320
                MM_PER_PIXEL = 0.875
                
                real_x = round(( ROBOT_BASE_X_PIXEL -cx) * MM_PER_PIXEL, 1)
                
                # Y-অ্যাক্সিস উল্টানো (Flipped)
                real_y = round((cy - ROBOT_BASE_Y_PIXEL) * MM_PER_PIXEL, 1)
                
                object_found = True # অবজেক্ট পাওয়া গেছে
                
                cv2.drawContours(frame, [biggest_contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                text = f"X:{real_x}mm, Y:{real_y}mm"
                cv2.putText(frame, text, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("SCARA Robot Eye", frame)
    #cv2.imshow("Color Mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
        
    # =================================================================
    # NEW: SEND COMMAND TO ESP32 WHEN 'p' IS PRESSED
    # =================================================================
    elif key == ord('p'):
        if object_found:
            # ESP32 কে পাঠানোর জন্য কমান্ড তৈরি করা (যেমন: "VPICK 39.5,82.0\n")
            command = f"VPICK {real_x},{real_y}\n"
            esp32.write(command.encode('utf-8'))
            print(f"Sent to Robot: {command.strip()}")
        else:
            print("No yellow object in view to pick!")

cap.release()
esp32.close()
cv2.destroyAllWindows()