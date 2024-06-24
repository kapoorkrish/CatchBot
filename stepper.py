import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time

################################
# RPi and Motor Pre-allocations
################################

# Declare a instance of class pass GPIO pins numbers and the motor type
motor1 = RpiMotorLib.A4988Nema(20, 21, (22,22,22), "DRV8825")
motor2 = RpiMotorLib.A4988Nema(23, 24, (22,22,22), "DRV8825")

###########################
# Actual motor control
###########################
#
start_time = time.time()
motor1.motor_go(True, # False=Clockwise, True=Counterclockwise
                "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                (200) * 5, # number of steps
                .0005, # step delay [sec]
                False, # True = print verbose output 
                .05) # initial delay [sec]
print(time.time() - start_time)
motor2.motor_go(False, # False=Clockwise, True=Counterclockwise
                "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                (200) * 5, # number of steps
                .0005, # step delay [sec]
                False, # True = print verbose output 
                .05) # initial delay [sec]

GPIO.cleanup() # clear GPIO allocations after run