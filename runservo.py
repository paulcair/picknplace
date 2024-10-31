"""
runservo.py

This is a script to control leArm 6DOF robot via serial connection. This script is writtent to run through each servo one at a time in order to determine the range of motion for each servo and to defined the Denavit-Hartenberg parameters for each joint.

This script is used to control servos via a serial connection. It includes functions to:
- Get the battery voltage of the connected device.
- Run predefined action groups.
- Move servos to specified angles over a given time.

The script initializes a serial connection, defines action arrays, and iterates over servos to perform movements.

Author: [Your Name]
Date: [Date]
"""

import serial
import time

#create action array each row is a command to a servo with {servo_id, angle}
action = [
    {'servo_id': 1, 'angle': 2200},
    {'servo_id': 2, 'angle': 500},
    {'servo_id': 3, 'angle': 500},
    {'servo_id': 4, 'angle': 500},
    {'servo_id': 5, 'angle': 500},
    {'servo_id': 6, 'angle': 500}
]

print("Starting servo control")
# Open serial connection (adjust port and baudrate as needed)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

def get_battery_voltage():
    # Packet to request battery voltage
    packet = [0x55, 0x55, 0x02, 0x0F]  # Header + Length + Command (CMD_GET_BATTERY_VOLTAGE)
    ser.write(bytearray(packet))
    response = ser.read(6)  # Reading the response packet
    decode_battery_voltage(response)

def decode_battery_voltage(response):
    # Check if response is valid and starts with 'UU' (0x55 0x55)
    if response[:2] == b'\x55\x55':
        lower_voltage = response[4]  # 5th byte
        higher_voltage = response[5]  # 6th byte
        
        # Calculate battery voltage in millivolts
        voltage = (higher_voltage << 8) + lower_voltage
        print(f"Battery voltage: {voltage} mV ({voltage / 1000:.3f} V)")
    else:
        print("Invalid response")


def run_action_group(ser, group_number, run_times, estimated_time):
    # Split run_times into lower and higher bytes
    lower_time = run_times & 0xFF       # Lower 8 bits
    higher_time = (run_times >> 8) & 0xFF  # Higher 8 bits
    
    # Packet format: [0x55, 0x55, Length, Command, Param 1, Param 2, Param 3]
    packet = [
        0x55, 0x55,  # Frame header
        0x05,        # Data length (always 5 for CMD_FULL_ACTION_RUN)
        0x06,        # Command (CMD_FULL_ACTION_RUN)
        group_number,  # Parameter 1: Action group number
        lower_time,    # Parameter 2: Lower 8 bits of run_times
        higher_time    # Parameter 3: Higher 8 bits of run_times
    ]
    
    # Send the packet
    ser.write(bytearray(packet))
    print(f"Sent action group {group_number} to run {run_times} times.")
    print("Sent packet:", packet)
    print(f"Waiting for: {estimated_time} s")
    time.sleep(estimated_time)

def move_servo(action_array, time_ms):
    # Command parameters
    command = 0x03  # CMD_MULT_SERVO_MOVE
    num_servos = len(action_array)
      
    # Length is num_servos * 3 (ID, angle low, angle high) + 5 (header and time)
    length = num_servos * 3 + 5

    # Time split into lower and higher bytes
    time_low = time_ms & 0xFF
    time_high = (time_ms >> 8) & 0xFF

    # Create the command packet
    packet = [
        0x55, 0x55,          # Frame header
        length,              # Data length
        command,             # Command
        num_servos,          # Number of servos
        time_low, time_high # Time to complete action
    ]
    
    # Go through the action array and append the servo ID and angle bytes to the packet
    for action in action_array:      
        # Angle split into lower and higher bytes
        angle_low = action['angle'] & 0xFF
        angle_high = (action['angle'] >> 8) & 0xFF

        # Append servo ID and angle bytes to the packet
        packet.extend([action['servo_id'], angle_low, angle_high])
    
    print("Sent packet:", packet)
    # Convert to byte array and send the packet
    ser.write(bytearray(packet))
    print(f"Waiting for: {time_ms / 1000 +1} s")
    time.sleep(time_ms / 1000 + 1)
    

    

# Scripts to run
get_battery_voltage()


# Define the angle ranges for each servo
angle_ranges = {
    1: range(1500, 2501, 500),  # Servo 1: 1500 to 2500
    2: range(500, 2501, 500),   # Servo 2: 500 to 2500
    3: range(500, 2501, 500),   # Servo 3: 500 to 2500
    4: range(500, 2501, 500),   # Servo 4: 500 to 2500
    5: range(500, 2501, 500),   # Servo 5: 500 to 2500
    6: range(500, 2501, 500)    # Servo 6: 500 to 2500
}

move_servo(action_array=action, time_ms=500)
#run_action_group(ser, group_number=0, run_times=1, estimated_time=1)

# Iterate over servos from 6 down to 1
for servo_id in range(2, 0, -1):
    # Iterate over the angle range for the current servo
    for angle in angle_ranges[servo_id]:
        # Update the action array for the current servo
        this_action = [{'servo_id': servo_id, 'angle': angle}]

        if servo_id != 1:
            this_action.append({'servo_id': 1, 'angle': 2200})
        
        # Move the servo to the current angle
        move_servo(action_array=this_action, time_ms=100)
        move_servo(action_array=action, time_ms=1000)
        
    

# Close the serial connection when done
ser.close()