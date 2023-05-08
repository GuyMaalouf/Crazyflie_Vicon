# Crazyflie Drone Control Using Vicon Vision System

## Introduction
This repository contains three Python codes that are designed to control the Crazyflie drone using the Vicon vision system. The Vicon vision system is used to detect the position of the drone and send the information through a UDP stream over Wi-Fi to a PC. The PC then sends commands to the Crazyflie drone using the Crazyradio. The three codes are named as follows:

0_fwd_bwd_V1.py: This code makes the drone move forward and backward until it reaches a preset limit. The position of the drone is detected by the Vicon system.

1_fwd_bwd_V2.py: This is an improved version of code 0_fwd_bwd_V1.py where the positions are stored in a global variable. This code has the ability to store the takeoff position and use it to find the relative positions of the drone (i.e., the position of the drone relative to its takeoff point, not to the center of the Vicon system).

2_square.py: This code makes the drone bounce within a virtual box. The position of the drone is read by the Vicon system and fed into the PC through a UDP stream sent over Wi-Fi.

## Requirements
Crazyflie 2.0 drone
Vicon Vision System
Crazyradio
Python 3.5


Vicon DataStream SDK
Crazyflie Python Library
Numpy Library

## How to use
1- Connect your PC to the same Wi-Fi network as the Vicon Vision System.

2- Connect the Crazyradio to your PC.

3- Turn on the Crazyflie drone and place it in the Vicon motion capture area.

4- Run the desired Python code in your terminal or IDE.

5- The drone will move according to the instructions in the code.

6- To stop the drone, press Ctrl+C in the terminal.
