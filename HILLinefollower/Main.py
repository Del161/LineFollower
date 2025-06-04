from machine import Pin, UART
from time import sleep
import heapq
from Dijkstra import dijkstra

led = Pin(2, Pin.OUT)     # Define ESP32 onboard LED
button_left = Pin(5, Pin.IN, Pin.PULL_DOWN)

led.value(0) # turn off the red led on start

start = (10, 16)
goal = (0,0)
path = dijkstra(start, goal)
print(path ," here is the shortest path")

# end dijkstra ################################################# end dijkstra

x = 0
y = len(path)

## this lets you stop it wahoo
print("waiting")
while button_left() == True:
    sleep(0.25)
    led.value(not led())
#     if x+1 < y:
#         old_direction = path[x]
#         new_direction = path[x+1]
#         directionection = (old_direction[0] - new_direction[0], old_direction[1] - new_direction[1])  
#         print(directionection, x)
#         x+=1
    
print("starting")
led.value(1)

# Set serial to UART1 using the same pins as UART0 to communicate via USB
uart = UART(1, 115200, tx=1, rx=3)

# Initial status of the line sensor: updated by Webots via serial
line_left = False
line_center = False
line_right = False

# Variables to implement the line-following state machine
current_state = 'forward'
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50
state_updated = True

direct = "left"

while True:
    
    if uart.any():
        msg_bytes = uart.read()    # Read all received messages
        msg_str = str(msg_bytes, 'UTF-8')  # Convert to string
        # Ignore old messages (Webots can send faster than the ESP32 can process)
        # Then split them in the same order used in Webots and update sensor status

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

    ##################   Think   ###################

    # Implement the line-following state machine transitions
    if current_state == 'forward':
        counter = 0
        if line_right and not line_left and not line_center or line_right and not line_left and direct == "right":
            current_state = 'turn_right'
            state_updated = True
            led.value(1)
        elif line_left and not line_right and not line_center or line_left and not line_right and direct == "left":
            current_state = 'turn_left'
            state_updated = True
            led.value(1)
        elif line_left and line_right and line_center: # lost the line
            current_state = 'turn_left'
            state_updated = True
            led.value(1)
            
    if current_state == 'turn_right':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
            led.value(1)

    if current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
            led.value(1)

    if current_state == 'stop':
        led.value(1)
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            state_update = True
            led.value(1)
            
    
    ##################   Act   ###################

    # Send the new state when updated
    if state_updated == True:
        uart.write(current_state + '\n')
        state_updated = False
        led.value(0)

    counter += 1    # increment counter
    sleep(0.02)     # wait 0.02 seconds
