# picknplace
Repository for scrips to run a 6DOF arm with objective to have an AI picknplace of 3D printed cubes using openCV to detect objects and find their coordinates

## How to operate

### Step 1: Power on the robot and determine the port it is connected to

Power on the robot, then open a terminal and run this command (for linux)

```
ls -l /dev/ttyUSB*
```

Look at the printed out list to find the port the robot is connected to and copy the

### Step 2: Connect to the robot

open commandRobot.py and update line 25 "port" value (in this case it is /dev/ttyUSB0) 

```
# Define the serial parameters
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.ser = self.initialize_serial(port, baudrate, timeout)
```

### Step 3: Command the robot
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
and run the following script to put the robot in a given position

```
sudo python3 forwardKinematics.py
```

you may have to input your password for sudo command