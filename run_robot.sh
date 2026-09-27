#!/bin/bash

# Set default values
THETA_1=${1:-90}
THETA_2=${2:-90}
THETA_3=${3:-0}
THETA_4=${4:-0}
THETA_5=${5:-90}
TOOL_X=${6:-30}

# Check if user is in dialout group
if ! groups | grep -q "dialout"; then
  echo "Warning: You're not in the dialout group. Adding you now..."
  sudo usermod -a -G dialout $USER
  echo "Please restart your session for changes to take effect."
fi

# Update joint angles in forwardKinematics.py
sudo sed -i "s/theta_1_deg = .*/theta_1_deg = $THETA_1/" /home/paul-cairns/Documents/programming/picknplace/forwardKinematics.py
sudo sed -i "s/theta_2_deg = .*/theta_2_deg = $THETA_2/" /home/paul-cairns/Documents/programming/picknplace/forwardKinematics.py
sudo sed -i "s/theta_3_deg = .*/theta_3_deg = $THETA_3/" /home/paul-cairns/Documents/programming/picknplace/forwardKinematics.py
sudo sed -i "s/theta_4_deg = .*/theta_4_deg = $THETA_4/" /home/paul-cairns/Documents/programming/picknplace/forwardKinematics.py
sudo sed -i "s/theta_5_deg = .*/theta_5_deg = $THETA_5/" /home/paul-cairns/Documents/programming/picknplace/forwardKinematics.py
sudo sed -i "s/tool_x = .*/tool_x = $TOOL_X/" /home/paul-cairns/Documents/programming/picknplace/forwardKinematics.py

# Run forward kinematics calculation with sudo
sudo python3 /home/paul-cairns/Documents/programming/picknplace/forwardKinematics.py