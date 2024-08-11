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

X_max = 1450
Y_max = 1600
min = 0

# X_pos = 0
# Y_pos = 0
X_pos = int(X_max / 2)
Y_pos = int(Y_max / 2)

# Options
speed = (.0005) * 1

# Movement
t1 = threading.Thread()
t2 = threading.Thread()

def move_motor(motor: RpiMotorLib.A4988Nema, dir: bool, move: int, tps: float):
    motor.motor_go(dir, "Full" , move, tps, False, .05)

def to_pos(X: int, Y: int, tps=speed):
    global X_pos
    global Y_pos
    global X_max
    global Y_max

    # Calculate how much to move to reach position
    X_move = X - X_pos
    Y_move = Y - Y_pos

    # Set X axis position, ensure in bounds
    if (X_pos + X_move) > X_max:
        X_move = X_max - X_pos
        X_pos = X_max
    elif (X_pos + X_move) < min:
        X_move = min - X_pos
        X_pos = min
    else:
        X_pos += X_move
    
    # Set Y axis position, ensure in bounds
    if (Y_pos + Y_move) > Y_max:
        Y_move = Y_max - Y_pos
        Y_pos = Y_max
    elif (Y_pos + Y_move) < min:
        Y_move = min - Y_pos
        Y_pos = min
    else:
        Y_pos += Y_move

    # Calculate motor position
    motor1_pos = 0
    motor2_pos = 0
    
    motor1_pos = X_move + Y_move
    motor2_pos = X_move - Y_move

    # Calculate motor direction
    motor1_dir = True
    motor2_dir = True

    if (motor1_pos < 0):
        motor1_dir = False
    if (motor2_pos < 0):
        motor2_dir = False

    t1 = threading.Thread(target=move_motor, args=(motor1, motor1_dir, abs(motor1_pos), tps))
    t2 = threading.Thread(target=move_motor, args=(motor2, motor2_dir, abs(motor2_pos), tps))
    
    t1.start()
    t2.start()
    t1.join()
    t2.join()

    print(f"({X}, {Y})")
