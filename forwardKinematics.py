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

#input desired joint angles (0-180) in degrees and desired claw width (0<x<30)
theta_1_deg = 90
theta_2_deg = 90
theta_3_deg = 90
theta_4_deg = 90
theta_5_deg = 90
tool_x = 15

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


