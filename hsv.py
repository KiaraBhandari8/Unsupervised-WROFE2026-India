from picamera2 import Picamera2
import cv2
import numpy as np
import time


class HSVRangeHighlighter:
    def __init__(self):
        self.window_name = "HSV Tuner"

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1200, 800)

        cv2.createTrackbar("HUE MIN", self.window_name, 0, 179, lambda x: None)
        cv2.createTrackbar("HUE MAX", self.window_name, 179, 179, lambda x: None)

        cv2.createTrackbar("SAT MIN", self.window_name, 0, 255, lambda x: None)
        cv2.createTrackbar("SAT MAX", self.window_name, 255, 255, lambda x: None)

        cv2.createTrackbar("VAL MIN", self.window_name, 0, 255, lambda x: None)
        cv2.createTrackbar("VAL MAX", self.window_name, 255, 255, lambda x: None)

    def apply_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("HUE MIN", self.window_name)
        h_max = cv2.getTrackbarPos("HUE MAX", self.window_name)

        s_min = cv2.getTrackbarPos("SAT MIN", self.window_name)
        s_max = cv2.getTrackbarPos("SAT MAX", self.window_name)

        v_min = cv2.getTrackbarPos("VAL MIN", self.window_name)
        v_max = cv2.getTrackbarPos("VAL MAX", self.window_name)

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        mask = cv2.inRange(hsv, lower, upper)

        result = cv2.bitwise_and(image, image, mask=mask)

        text = (
            f"Lower HSV: [{h_min}, {s_min}, {v_min}]   "
            f"Upper HSV: [{h_max}, {s_max}, {v_max}]"
        )

        cv2.putText(
            result,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        return result

    def run(self, image):
        while True:
            result = self.apply_mask(image)

            cv2.imshow(self.window_name, result)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                break

            elif key == ord("p"):
                h_min = cv2.getTrackbarPos("HUE MIN", self.window_name)
                h_max = cv2.getTrackbarPos("HUE MAX", self.window_name)

                s_min = cv2.getTrackbarPos("SAT MIN", self.window_name)
                s_max = cv2.getTrackbarPos("SAT MAX", self.window_name)

                v_min = cv2.getTrackbarPos("VAL MIN", self.window_name)
                v_max = cv2.getTrackbarPos("VAL MAX", self.window_name)

                print("\n===== HSV RANGE =====")
                print(f"LOWER = [{h_min}, {s_min}, {v_min}]")
                print(f"UPPER = [{h_max}, {s_max}, {v_max}]")

        cv2.destroyAllWindows()


def capture_image():
    print("Initializing camera...")

    picam2 = Picamera2()

    config = picam2.create_still_configuration(
        main={"size": (1280, 720)}
    )

    picam2.configure(config)
    picam2.start()

    print("Waiting for camera adjustment...")
    time.sleep(2)

    print("Capturing image...")
    image = picam2.capture_array()

    picam2.stop()

    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    cv2.imwrite("captured_image.jpg", image)

    print("Image saved as captured_image.jpg")

    return image


if __name__ == "__main__":
    print("\nHSV TUNER")
    print("ESC = Exit")
    print("P = Print HSV values\n")

    image = capture_image()

    tuner = HSVRangeHighlighter()
    tuner.run(image)