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

X_pos = 0
Y_pos = 0

# Options
speed = (.0005) * 1
X_max = 1450
Y_max = 1600
min = 0

# Movement
t1 = threading.Thread()
t2 = threading.Thread()

def check_pos(is_X: bool, dir: bool, pos: int):
    global X_pos
    global Y_pos
    global X_max
    global Y_max
    current_pos = 0
    max = 0

    # Read axis position and bounds
    if is_X:
        current_pos = X_pos
        max = X_max
    else:
        current_pos = Y_pos
        max = Y_max

    # Set axis position, ensure in bounds
    if (current_pos + pos) > max and dir:
        pos = max - current_pos
        current_pos = max
    elif (current_pos - pos) < min and not dir:
        pos = current_pos - min
        current_pos = min
    else:
        if not dir:
            current_pos -= pos
        else:
            current_pos += pos

    # Update axis position
    if is_X:
        X_pos = current_pos
    else:
        Y_pos = current_pos
    
    # Return safe position in bounds
    return pos

def move_motor(motor: RpiMotorLib.A4988Nema, dir: bool, pos: int):
    motor.motor_go(dir, "Full" , pos, speed, False, .05)

def X(dir: bool, pos: int):
    pos = check_pos(True, dir, pos)

    t1 = threading.Thread(target=move_motor, args=(motor1, dir, pos))
    t2 = threading.Thread(target=move_motor, args=(motor2, dir, pos))
    
    t1.start()
    t2.start()
    t1.join()
    t2.join()

def Y(dir: bool, pos: int):
    pos = check_pos(False, dir, pos)

    t1 = threading.Thread(target=move_motor, args=(motor1, dir, pos))
    t2 = threading.Thread(target=move_motor, args=(motor2, not dir, pos))
    
    t1.start()
    t2.start()
    t1.join()
    t2.join()

Y(True, int(Y_max / 2))
X(True, int(X_max / 2))

GPIO.cleanup()