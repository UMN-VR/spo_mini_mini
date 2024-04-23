from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)


# Set the servo on channel 0 to 90 degrees
kit.servo[0].angle = 80


# wait for 1 second
import time
time.sleep(1)

# Set the servo on channel 0 to 90 degrees
kit.servo[0].angle = 70


# wait for 1 second
import time
time.sleep(1)

# Set the servo on channel 0 to 90 degrees
kit.servo[0].angle = 90

# wait for 1 second
import time
time.sleep(1)

# deactivating the servo
kit.servo[0].angle = None
