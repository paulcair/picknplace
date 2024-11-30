"""
commandRobot.py

This is a script to control leArm 6DOF robot via serial connection. This script is designed to simply command the robot from other scripts.

This script is used to control servos via a serial connection. It includes functions to:
- Get the battery voltage of the connected device.
- Run predefined action groups.
- Move servos to specified angles over a given time.

The script initializes a serial connection, defines action arrays, and iterates over servos to perform movements.

Author: Paul Cairns
Date: Nov 30th, 2024
"""

import serial
import time

class RobotController:
    # Define the serial parameters
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.ser = self.initialize_serial(port, baudrate, timeout)

    # Function to initialize the serial connection
    def initialize_serial(self, port, baudrate, timeout):
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
            print("Serial connection established.")
            return ser
        except serial.SerialException as e:
            print(f"Failed to establish serial connection: {e}")
            return None

    def get_battery_voltage(self):
        if not self.ser:
           print("No serial connection available.")
           return
        try:
            # Packet to request battery voltage
            packet = [0x55, 0x55, 0x02, 0x0F]  # Header + Length + Command (CMD_GET_BATTERY_VOLTAGE)
            self.ser.write(bytearray(packet))
            response = self.ser.read(6)  # Reading the response packet
            self.decode_battery_voltage(response)
        except Exception as e:
            print(f"Error getting battery voltage: {e}")

    def decode_battery_voltage(self, response):
        # Check if response is valid and starts with 'UU' (0x55 0x55)
        if response[:2] == b'\x55\x55':
            lower_voltage = response[4]  # 5th byte
            higher_voltage = response[5]  # 6th byte
            
            # Calculate battery voltage in millivolts
            voltage = (higher_voltage << 8) + lower_voltage
            print(f"Battery voltage: {voltage} mV ({voltage / 1000:.3f} V)")
        else:
            print("Invalid response")


    def run_action_group(self, group_number, run_times, estimated_time):
        if not self.ser:
            print("No serial connection available.")
            return
        
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
        self.ser.write(bytearray(packet))
        print(f"Sent action group {group_number} to run {run_times} times.")
        print("Sent packet:", packet)
        print(f"Waiting for: {estimated_time} s")
        time.sleep(estimated_time)

    def move_servo(self, action_array, time_ms):
        if not self.ser:
            print("No serial connection available.")
            return
        
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
        
        #print("Sent packet:", packet)
        # Convert to byte array and send the packet
        self.ser.write(bytearray(packet))
        #print(f"Waiting for: {time_ms / 1000 +1} s")
        time.sleep(time_ms / 1000 + 1)
        
    def move(self, angles):
        if not self.ser:
            print("No serial connection available.")
            return
        
        try:
            #create action array each row is a command to a servo with {servo_id, angle}
            action = [
                {'servo_id': 1, 'angle': angles[0]},
                {'servo_id': 2, 'angle': angles[1]},
                {'servo_id': 3, 'angle': angles[2]},
                {'servo_id': 4, 'angle': angles[3]},
                {'servo_id': 5, 'angle': angles[4]},
                {'servo_id': 6, 'angle': angles[5]}
            ]
            self.move_servo(action_array=action, time_ms=500)
        except Exception as e:
            print(f"Error moving servos: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")    


# Example usage
if __name__ == "__main__":
    robot = RobotController()
    robot.get_battery_voltage()
    robot.move([2200, 500, 500, 500, 500, 500])
    robot.close()