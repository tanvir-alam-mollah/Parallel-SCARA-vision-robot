print("SCARA vision Code: Object Tracking")
import cv2
import numpy as np # <-- NEW: We need numpy to handle arrays of color values

camera_url = "http://192.168.0.181:8080/video"

print("Connecting to camera...")
cap = cv2.VideoCapture(camera_url)

if not cap.isOpened():
    print("Error: Could not connect to the camera.")
    exit()

print("Camera connected! Press 'q' on your keyboard to close the window.")

while True:
    success, frame = cap.read()
    
    if not success:
        print("Lost connection to camera.")
        break
        
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)  # Rotate the frame if needed
    

    # =================================================================
    # NEW VISION PROCESSING CODE STARTS HERE
    # =================================================================
    
    # 1. Convert the image from BGR (standard) to HSV (Hue, Saturation, Value)
    # HSV is much better for finding colors because it separates the color from the brightness.
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Define the color range for a YELLOW object
    # Hue for yellow is usually between 20 and 40 in OpenCV.
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])

    # 3. Create a "mask". This turns everything yellow into pure WHITE.
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    


    # 4. Find the contours (outlines) of the white shapes in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # If we found any contours...
    if len(contours) > 0:
        # 5. Pick the largest contour (we assume the biggest blue thing is our object)
        biggest_contour = max(contours, key=cv2.contourArea)
        
        # Only track it if it's large enough
        if cv2.contourArea(biggest_contour) > 500:
            
            # 6. Calculate the center pixel (cx, cy)
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # =================================================================
                # NEW: CALIBRATION MATH (PIXELS TO MILLIMETERS)
                # =================================================================
                # UPDATE THESE THREE NUMBERS BASED ON YOUR MEASUREMENTS:
                ROBOT_BASE_X_PIXEL = 243  # Pixel X where your robot's base is
                ROBOT_BASE_Y_PIXEL = 320  # Pixel Y where your robot's base is
                MM_PER_PIXEL = 0.875        # How many mm one pixel represents

                print(f"Center Pixels -> X: {cx}, Y: {cy}")
                # Calculate real world coordinates
                # X is right/left of the base, Y is straight out from the base
                real_x = round((ROBOT_BASE_X_PIXEL - cx) * MM_PER_PIXEL, 1)
                
                # We subtract cy from the base Y because pixel Y goes DOWN, 
                # but real-world Y goes UP (away from the robot)
                # cy থেকে ROBOT_BASE_Y_PIXEL বিয়োগ করলে সাইন (+/-) ঠিক উল্টে যাবে
                real_y = round((cy - ROBOT_BASE_Y_PIXEL) * MM_PER_PIXEL, 1)
                
                # Round to 1 decimal place for the ESP32
                #real_x = round(real_x, 1)
                real_y = round(real_y, 1)
                
                # =================================================================

                # 7. Draw graphics on the screen
                cv2.drawContours(frame, [biggest_contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                
                # Print the REAL WORLD coordinates onto the video feed!
                text = f"X:{real_x}mm, Y:{real_y}mm"
                cv2.putText(frame, text, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # =================================================================
    # NEW VISION PROCESSING CODE ENDS HERE
    # =================================================================

    # Show the normal video feed
    cv2.imshow("SCARA Robot Eye", frame)
    
    # Also show the Mask so you can see exactly how the computer isolates the color!
    cv2.imshow("Color Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()