# LineFollower
A line following robot using webots and an esp32 using microPython

Boot.py just runs the Main.py, which contains the primary functions.

Main.py communicates with the webots application and decides the direction / state of the machine.

Dijkstra.py contains a micropython compatible dijkstra algorithm that calculates the fastest possible route across 
a map plotted in a grid.

The esp32 part of the code has dependencies on heapq, time, and machine
The webots part of the code uses Numpy, heapq, matplotlib, and controller.

All of the code is on python 3.6.9, and webots R2025a
