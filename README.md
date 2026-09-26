# picknplace
Repository for scrips to run a 6DOF arm with objective to have an AI picknplace of 3D printed cubes using openCV to detect objects and find their coordinates

## How to operate

Go to forwardKinematics.py and input the desired joint angles in the section below:

```
Input desired joint angles (0-180) in degrees and desired claw width (0<x<30)
theta_1_deg = 90
theta_2_deg = 90
theta_3_deg = 0
theta_4_deg = 0
theta_5_deg = 90
tool_x = 30
```
and run the script to put the robot in a given position