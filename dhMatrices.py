import numpy as np
import math

def dh_transform(theta, d, a, alpha):
    """
    Create the D-H transformation matrix.
    
    Parameters:
    - theta: Joint angle (in radians)
    - d: Link offset
    - a: Link length
    - alpha: Link twist (in radians)
    
    Returns:
    - A 4x4 transformation matrix
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def get_dh_matrices(theta_1, theta_2, theta_3, theta_4, theta_5, S_1):
    """
    Calculate the D-H transformation matrices for the robot based on the input joint angles.
    
    Parameters:
    - theta_1, theta_2, theta_3, theta_4, theta_5: Joint angles in radians
    - S_1: end effector position value (Servo 1)
    
    Returns:
    - A list of D-H transformation matrices
    """
    # Define the D-H parameters for each joint
    d = [95, 0, 0, 0, 150, 0]  # Link offsets
    a = [0, 105, 98, 0, 0, 0]  # Link lengths
    alpha = [math.radians(90), math.radians(180), 0, math.radians(90), math.radians(90), 0]  # Link twists
    
    # Create the transformation matrices
    T1 = dh_transform(theta_1 + math.radians(180), d[0], a[0], alpha[0])
    T2 = dh_transform(theta_2 + math.radians(180), d[1], a[1], alpha[1])
    T3 = dh_transform(theta_3 + math.radians(90), d[2], a[2], alpha[2])
    T4 = dh_transform(theta_4, d[3], a[3], alpha[3])
    T5 = dh_transform(theta_5, d[4], a[4], alpha[4])
    T6 = np.array([
        [1,0,0,0],
        [0,1,0,(S_1-1500)*0.03+55],
        [0,0,1,0],
        [0,0,0,1]
    ])

    return [T1, T2, T3, T4, T5, T6]