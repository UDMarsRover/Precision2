#!/bin/bash

# Start Bluetooth control
bluetoothctl << EOF
power on
agent on
default-agent
pairable on
scan on
EOF

# Wait for 5 seconds
sleep 5

# Attempt to connect to the device
bluetoothctl connect <DEVICE_MAC_ADDRESS>

# Check if the device is connected
if bluetoothctl info <DEVICE_MAC_ADDRESS> | grep -q "Connected: yes"; then
    echo "Device connected successfully."
else
    echo "Device not found. Exiting."
    exit 1
fi

# 98:7A:14:89:A6:F8