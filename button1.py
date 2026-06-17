from gpiozero import Button
import time
import subprocess


# Function to be called when the button is pressed
def run_script():
    print("Button is PRESSED. Running openround1.py...")
    try:
    #  subprocess.run(['/home/pi8/wrofe2025/env_test/bin/python', '/home/pi8/wrofe2025/openround1_cam_02sep.py'], check=True)
     subprocess.run(['/home/pi8/wrofe2025/env_test/bin/python', '/home/pi8/wrofe2025/obs_main_combine_opt_04sept.py'], check=True)

    except Exception as e:
        print(f"Error running openround1.py: {e}")
    print("Script finished running.\n")


# Set up the button
button = Button(26)


# Assign the function to the 'when_pressed' event
button.when_pressed = run_script


print("Starting program. Monitoring button state... Press CTRL+C to exit.\n")


# Keep the script running indefinitely
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nProgram stopped.")