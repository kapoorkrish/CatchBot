import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import threading

# Motor control
# False=Clockwise, True=Counterclockwise
# Step type (Full,Half,1/4,1/8,1/16,1/32)
# number of steps
# step delay [sec]
# True = print verbose output 
# initial delay [sec]

# Initialize
motor1 = RpiMotorLib.A4988Nema(20, 21, (22,22,22), "DRV8825")
motor2 = RpiMotorLib.A4988Nema(23, 24, (22,22,22), "DRV8825")

motor1_pos = 0
motor2_pos = 0

# Options
speed = (.0005) * 2
max = 1000
min = 0

# Movement
def move_motor(motor: RpiMotorLib.A4988Nema, dir: bool, pos: int):
    global motor1_pos
    global motor2_pos

    # Read motor position
    if motor == motor1:
        motor_pos = motor1_pos
    else:
        motor_pos = motor2_pos

    # Set motor position, ensure in bounds
    if (motor_pos + pos) > max and dir:
        pos = max - motor1_pos
        motor_pos = max
    elif (motor_pos - pos) < min and not dir:
        pos = motor_pos - min
        motor_pos = min
    else:
        if not dir:
            motor_pos -= pos
        else:
            motor_pos += pos

    # Update motor position
    if motor == motor1:
        motor1_pos = motor_pos
    else:
        motor2_pos = motor_pos
    
    motor.motor_go(dir, "Full" , pos, speed, False, .05)

t1 = threading.Thread()
t2 = threading.Thread()

def X(dir: bool, pos: int):
    t1 = threading.Thread(target=move_motor, args=(motor1, dir, pos))
    t2 = threading.Thread(target=move_motor, args=(motor2, dir, pos))
    
    t1.start()
    t2.start()
    t1.join()
    t2.join()

def Y(dir: bool, pos: int):
    t1 = threading.Thread(target=move_motor, args=(motor1, dir, pos))
    t2 = threading.Thread(target=move_motor, args=(motor2, not dir, pos))
    
    t1.start()
    t2.start()
    t1.join()
    t2.join()

# X(True, 1000)
# X(True, 800)
# X(False, 500)
# X(False, 1000)

GPIO.cleanup()