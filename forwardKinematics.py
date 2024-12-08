"""
forwardKinematics.py

This is a script to control leArm 6DOF robot via serial connection. This script is written to perform forward kinematics and output the predicted (x,y,z) coordinates of the end effector.
This script is used to control servos via a serial connection. It includes functions to:
- Take in desired servo angles.
- Defines the D-H Transformation matrices and performs matrix multiplications to detirmine the x,y,z coordinates.
- Moves the servos and the end effector to the desired positions by calling runServo.py.

The script initializes a serial connection, defines action arrays, and iterates over servos to perform movements.

Author: Paul Cairns
Date: Nov 30th, 2024
"""
import math
import numpy as np
import commandRobot
from dhMatrices import get_dh_matrices

# Starting position
starting_position = [1500, 500, 1400, 1500, 2450, 1500]

# Input desired joint angles (0-180) in degrees and desired claw width (0<x<30)
theta_1_deg = 0
theta_2_deg = 0
theta_3_deg = 0
theta_4_deg = 0
theta_5_deg = 0
tool_x = 30

# Convert the inputs into radians for matrix multiplications 
theta_1 = math.radians(theta_1_deg)
theta_2 = math.radians(theta_2_deg)
theta_3 = math.radians(theta_3_deg)
theta_4 = math.radians(theta_4_deg)
theta_5 = math.radians(theta_5_deg)

# Convert the inputs into servo values by maping them from degrees to a value between 500 and 2500 that will be sent to the servo. Note that servos are defined in reverse order as the thetas.
S_1 = int((30-tool_x)*2000/60+1500)
S_2 = int(theta_5_deg/180*2000+500)
S_3 = int(theta_4_deg/180*2000+500)
S_4 = int(theta_3_deg/180*2000+500)
S_5 = int(theta_2_deg/180*2000+500)
S_6 = int(theta_1_deg/180*2000+500)    
angles = [S_1, S_2, S_3, S_4, S_5, S_6]

# Pass the theta values and servo 1 (claw) value to the dhMatrices.py script and store the transformation matrices in the variable returned array dh_matrices 
dh_matrices = get_dh_matrices(theta_1, theta_2, theta_3, theta_4, theta_5, S_1)

# Multiply the transformation matrices to get the resulting transfromation matrix from frame 0 to 6
T0_6 = dh_matrices[0] @ dh_matrices[1] @ dh_matrices[2] @ dh_matrices[3] @ dh_matrices[4] 
print(dh_matrices)
print(T0_6)

def main():
    robot = commandRobot.RobotController()

    # Check if the serial connection was established successfully
    if robot.ser is None:
        print("Failed to establish serial connection. Exiting.")
        return

    try:
        # Send the command to move the robot to the desired position.
        print("Moving Robot...")
        robot.move(angles)
        robot.move(starting_position)
    except Exception as e:
        print(f"Error during movement: {e}")
    finally:
        robot.close()

if __name__ == "__main__":
    main()