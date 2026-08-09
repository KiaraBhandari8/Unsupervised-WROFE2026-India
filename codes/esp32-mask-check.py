import cv2
import numpy as np
import sys

def get_pi5_gstreamer_pipeline(width=640, height=480, fps=30):
    """
    Creates a GStreamer pipeline string that safely pulls a frame 
    from the Pi 5's modern libcamerasrc framework into OpenCV BGR format.
    """
    return (
        f"libcamerasrc ! "
        f"video/x-raw, width={width}, height={height}, framerate={fps}/1 ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink drop=true sync=false"
    )

# Array to store our clicked points
points = []

def click_event(event, x, y, flags, params):
    """Callback function that registers mouse clicks on the video frame."""
    global img
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append([x, y])
        print(f"Recorded Point {len(points)}: [{x}, {y}]")
        
        # Draw visual crosshairs/circles on screen where you clicked
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(img, f"P{len(points)}:[{x},{y}]", (x + 10, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow("Pi 5 Arena Calibration", img)
        
        if len(points) == 4:
            print("\n=======================================================")
            print("    SUCCESS! COPY THESE VALUES INTO YOUR MAIN CODE     ")
            print("=======================================================")
            print(f"top_left     = {points[0]}")
            print(f"top_right    = {points[1]}")
            print(f"bottom_right = {points[2]}")
            print(f"bottom_left  = {points[3]}")
            print("=======================================================\n")
            print("Press 'q' key on the image window to close.")

# Build pipeline and initialize the camera stream via GStreamer backend
pipeline_str = get_pi5_gstreamer_pipeline(width=640, height=480)
cap = cv2.VideoCapture(pipeline_str, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("\n[ERROR] Unable to hook into GStreamer pipeline!")
    print("Your OpenCV installation might be missing the GStreamer backend compile flag.")
    print("Attempting native fallback snapshot method...")
    # Clean fallback option if GStreamer is missing inside your env_test bin
    import subprocess
    subprocess.run(["rpicam-still", "-o", "/home/pi8/wrofe2025/fallback.jpg", "-t", "500", "--width", "640", "--height", "480", "-n"])
    img = cv2.imread("/home/pi8/wrofe2025/fallback.jpg")
    if img is None:
        print("[CRITICAL] Both streaming and snapshot methods failed.")
        sys.exit(1)
else:
    # Read past the initial camera warm-up buffer frames
    for _ in range(15):
        ret, img = cap.read()
    cap.release()

print("\nINSTRUCTIONS:")
print("Click the 4 corners of your 3m x 3m arena boundary in this exact order:")
print("1. Top-Left -> 2. Top-Right -> 3. Bottom-Right -> 4. Bottom-Left\n")

cv2.namedWindow("Pi 5 Arena Calibration")
cv2.setMouseCallback("Pi 5 Arena Calibration", click_event)
cv2.imshow("Pi 5 Arena Calibration", img)

# Keep graphical window active until 'q' is pressed
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()