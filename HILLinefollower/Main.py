"""
This is the main code to be ran on the esp32, it calculates its own path using a dijkstra algorithm
which it then follows using line sensor data from webots and a distance estimation
05-06-2025
"""

from machine import Pin, UART
from time import sleep
import heapq
from Dijkstra import dijkstra

led = Pin(2, Pin.OUT)     # Define ESP32 onboard LED
button_left = Pin(5, Pin.IN, Pin.PULL_DOWN)

led.value(0) # turn off the red led on start

# the starting postion of the robot Row, column<<<<<<<<<<<<<<<<<<<<<<
start = (11, 17)
# the goal / end position row,column
goal = (1,1)
# uses the dijkstra function from dijkstra.py to get the shortest path
path = dijkstra(start, goal)
print(path ," here is the shortest path, it is ", len(path), " steps")

x = 0
y = len(path)
startheading = "north"

## this lets you stop it the esp with a keyboardinterrupt
print("waiting")
while button_left() == True:
    sleep(0.25)
    led.value(not led())
#     if x+1 < y: # this is for debugging the dijkstra pathfinding
#         old_direction = path[x]
#         new_direction = path[x+1]
#         directionection = (old_direction[0] - new_direction[0], old_direction[1] - new_direction[1])  
#         print(directionection, x)
#         x+=1
    
print("starting")


# Set serial to UART1 using the same pins as UART0 to communicate via USB
uart = UART(1, 115200, tx=1, rx=3)

# Initial status of the line sensor is false
line_left = False
line_center = False
line_right = False

# Basic variables for changing the robots behaviour
# change the counter max for longer turns
current_state = 'forward'
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50
state_updated = True

# these are the headings the robot can have in relation to the camera
headings = ("north","east","south","west")
direct = "straight"
oldheading = startheading
newheading = startheading
dist = 0
path.append((0,0))

#this waits until the webots sim send communication to start to synchonize them
while not uart.any():
    led.value(not led())
    sleep(0.1)

# turn off led
led.value(0)

# this is the main loop
while True:
    
    if uart.any():
        msg_bytes = uart.read()    # Read all received messages
        msg_str = str(msg_bytes, 'UTF-8')  # Convert to string

        # this splits the recieved message into separate sensor data
        # line_left
        if msg_str[-4:-3] == '1':
            line_left = True
        else:
            line_left = False
        # line_center
        if msg_str[-3:-2] == '1':
            line_center = True
        else:
            line_center = False
        # line_right
        if msg_str[-2:-1] == '1':
            line_right = True
        else:
            line_right = False

    # The following part calculates the heading of the robot based on the given path
    # or if it is near the end of the path to stop
    if x + 1 >= y and dist >= 90:
        current_state = "stop"
        state_updated = True
        dist = 0
    
    elif dist >= 41 and x+1 < y:
        
        old_direction = path[x]
        new_direction = path[x+1]
        if old_direction[0] - new_direction[0] == 1:
            newheading = "north"
        elif old_direction[0] - new_direction[0] == -1:
            newheading = "south"
        
        elif old_direction[1] - new_direction[1] == 1:
            newheading = "west"
        elif old_direction[1] - new_direction[1] == -1:
            newheading = "east"
        x+=1
        dist = 0
        
        if oldheading == "north" and newheading == "west":
            direct = "left"
        elif oldheading == "north" and newheading == "east":
            direct = "right"
            
        elif oldheading == "west" and newheading == "north":
            direct = "right"
        elif oldheading == "west" and newheading == "south":
            direct = "left"
            
        elif oldheading == "east" and newheading == "south":
            direct = "right"
        elif oldheading == "east" and newheading == "north":
            direct = "left"
            
        elif oldheading == "south" and newheading == "east":
            direct = "left"
        elif oldheading == "south" and newheading == "west":
            direct = "right"
            
        elif oldheading == newheading:
            direct = "straight"
            
        state_updated = True
            
        # debugging statement that shows the heading and the direction it gives    
        #uart.write(oldheading + " "+ newheading +" "+ direct+ "\n")
    
    oldheading = newheading
    
    # The following part is the state machine deciding which turns to take and not to take
    # based on desired heading
    if current_state == 'forward':
        counter = 0
        
        """ These only turn right or left if there is a danger of losing the line, or if the current direction
        it needs to go in for the shortest path is right or left."""
        
        if line_right and not line_left and not line_center or line_right and direct == "right":
            current_state = 'turn_right'
            state_updated = True

        elif line_left and not line_right and not line_center or line_left and direct == "left":
            current_state = 'turn_left'
            state_updated = True
            
        # if all 3 sensors see white turn left until you run into it
        elif not line_left and not line_right and not line_center:
            current_state = 'turn_left'
            state_updated = True

    # turn right for the duration of the counter
    if current_state == 'turn_right':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
            dist -= 4

    # turn left for the duration of the counter
    if current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
            dist -= 4

    # stops at the goal
    if current_state == 'stop':
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            state_update = True
            break
    sleep(0.01) 
        
    # Send the new state when updated
    if state_updated == True:
        uart.write(current_state + " " + newheading + '\n')
        state_updated = False

    counter += 1    # increment counter for length of states
    dist += 1
    sleep(0.02)     # wait 0.02 seconds


