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
    

    # =================================================================
    # NEW VISION PROCESSING CODE STARTS HERE
    # =================================================================
    
    # 1. Convert the image from BGR (standard) to HSV (Hue, Saturation, Value)
    # HSV is much better for finding colors because it separates the color from the brightness.
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Define the color range for a BLUE object
    # If your object isn't blue, these numbers will need to change!
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])

    # 3. Create a "mask". This turns everything blue into pure WHITE, and everything else BLACK.
    mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

    # 4. Find the contours (outlines) of the white shapes in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # If we found any contours...
    if len(contours) > 0:
        # 5. Pick the largest contour (we assume the biggest blue thing is our object)
        biggest_contour = max(contours, key=cv2.contourArea)
        
        # Only track it if it's large enough (prevents the camera from tracking tiny blue specks)
        if cv2.contourArea(biggest_contour) > 500:
            
            # 6. Calculate the center pixel (cx, cy) of the object using "Image Moments"
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # 7. Draw graphics on the screen so we can see it working!
                cv2.drawContours(frame, [biggest_contour], -1, (0, 255, 0), 2) # Draw green outline
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)                # Draw red dot at center
                
                # Print the pixel coordinates onto the video feed
                text = f"X: {cx}, Y: {cy}"
                cv2.putText(frame, text, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Print to the console (This is the data we will eventually send to the ESP32!)
                # print(f"Object found at Pixel X: {cx}, Y: {cy}")

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