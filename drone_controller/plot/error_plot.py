# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import math

# Load the rosbag file
bag_file = raw_input('Enter the date of the rosbag file: ')
bag = rosbag.Bag('/home/agilicious/catkin_ws/src/rpg/DroneDelivery/agiclean/agiros/agiros/logs/drone-log-' + bag_file + '.bag')

# Define the desired positions
desired_x = -2  # Replace with your desired x position
desired_y = -1  # Replace with your desired y position
desired_yaw = 0  # Replace with your desired yaw position

# Initialize variables
current_x = None
current_y = None
current_yaw = None
error_x = None
error_y = None
error_yaw = None

# Function to convert quaternion to Euler angles
def calculate_euler_angles(x, y, z, w):
    # Calculate yaw (ψ)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    # Calculate pitch (θ)
    pitch = math.asin(2 * (w * y - x * z))

    # Calculate roll (φ)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x*x + y*y))

    return yaw, pitch, roll


# Initialize lists to store time and error values
time = []
error_x_values = []
error_y_values = []
error_yaw_values = []

# Iterate over the messages in the rosbag
for topic, msg, t in bag.read_messages(topics=['/mocap/bar_large/pose']): #/mocap/bar_large/pose
    current_x = msg.pose.position.x
    current_y = msg.pose.position.y
    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    yaw, pitch, roll = calculate_euler_angles(*quaternion)
    current_yaw = math.degrees(yaw) - 90
    # print('yaw: ', current_yaw)
    error_x = -(desired_x - current_x)
    error_y = -(desired_y - current_y)
    error_yaw = (desired_yaw - current_yaw)
    
    # Append time and error values to the lists
    time.append(t.to_sec())
    error_x_values.append(error_x)
    error_y_values.append(error_y)
    error_yaw_values.append(error_yaw)

# Create a figure and axis objects
fig, ax1 = plt.subplots()

# Plot x and y errors on the left y-axis
ax1.plot(time, error_x_values, 'b-', label='Error in x')
ax1.plot(time, error_y_values, 'g-', label='Error in y')
ax1.set_xlabel('Time')
ax1.set_ylabel('Error (x, y)')
ax1.set_ylim([-1, 5])
ax1.legend(loc='upper left')

# Create a second y-axis for yaw error
ax2 = ax1.twinx()
ax2.plot(time, error_yaw_values, 'r-', label='Error in yaw')
ax2.set_ylabel('Error (yaw)')
ax2.set_ylim([-360, 360])
ax2.legend(loc='upper right')

# Show the plot
plt.show()

# Close the rosbag
bag.close()