"""
Python code for communicatin with a hardware in the loop esp32
this code functions as the sensors and the motor drivers for 
the esp to communicate with
"""


from controller import Robot
import heapq
import matplotlib.pyplot as plt
import numpy as np
from Dijkstra_webot import dijkstra, plot_grid, create_grid

# the starting position of the robot on the grid and the
# grid position where you want to end up   
# row horizontal then column such as 11,13 11,17   
start =  (11, 13)
goal = (1, 1)

# for debugging plots
#grid = create_grid()
## calculate the path !!only for visual, not for actual pathfinding!!
#path = dijkstra(grid, start, goal)
## Visualize the Path overlayed on the map
#plot_grid(grid, path)
#plt.show()

#-------------------------------------------------------
# Open serial port to communicate with the microcontroller

import serial
try:
    # Change the port parameter according to your system
    ser =  serial.Serial(port='COM4', baudrate=115200, timeout=5) 
except:
    print("Communication failed. Check the cable connections and serial settings 'port' and 'baudrate'.")
    raise

# these dictate the speed of the robot
MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED
# create the Robot instance for the simulation.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

# states
states = ['forward', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'

#-------------------------------------------------------
# Initialize devices

# proximity sensors
# Ref.: https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python#understand-the-e-puck-model
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

direction = "straight"
#-------------------------------------------------------
# Main loop:
while robot.step(timestep) != -1:

    timestep = int(robot.getBasicTimeStep())   # [ms]
    delta_t = timestep/1000.0    # [s]

    # Update sensor readings
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Process sensor data
    line_right = gsValues[2] < 600
    line_center = gsValues[1] < 600
    line_left = gsValues[0] < 600
    
    # Build the message to be sent to the ESP32 with the ground
    # sensor data: 0 = line detected; 1 = line not detected
    message = ''
    if line_left:
        message += '1'
    else:
        message += '0'
    if line_center:
        message += '1'
    else:
        message += '0'
    if line_right:
        message += '1'
    else:
        message += '0'
    msg_bytes = bytes(message + '\n', 'UTF-8')
    

    # the following part of the code communicated with the esp32
    # Serial communication: if something is received, then update the current state
    if ser.in_waiting:
        value = str(ser.readline(), 'UTF-8')[:-1]  # ignore the last character
        messages = value.split()
        current_state = messages[0]
        direction =  messages[1]

    # the following code works as the motor drivers
    # Update speed according to the current state
    if current_state == 'forward':
        leftSpeed = speed
        rightSpeed = speed
            
    if current_state == 'turn_right':
        leftSpeed = 0.5 * speed
        rightSpeed = -0.4 * speed

    if current_state == 'turn_left':
        leftSpeed = -0.4 * speed
        rightSpeed = 0.5 * speed
        
    if current_state == 'stop':
        leftSpeed = 0.0
        rightSpeed = 0.0
        # this creates a map of the path taken after finishing
        # creates the grid
        grid = create_grid()
        # calculate the path !!only for visual, not for actual pathfinding!!
        path = dijkstra(grid, start, goal)
        # Visualize the Path overlayed on the map
        plot_grid(grid, path)
        plt.show()



    # this sets the speed of the motors based on the state from the esp
    # Update velocity commands for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
   
    # Print sensor message and current state for debugging
    print(f'Sensor message: {msg_bytes} - Current state: {current_state} - Direction: {direction}')

    # Send message to the microcontroller 
    ser.write(msg_bytes)  

ser.close()
