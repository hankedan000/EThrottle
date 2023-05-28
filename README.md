# Megasquirt Drive By Wire (msq-dbw)
This repo contains source code and PCB design files for a Megasquirt Extended CAN device that controls a drive by wire throttle body.

![block-diagram](/doc/block_diagram.svg)

# Features
 * tunable from Tuner Studio (via Megasquirt CAN passthrough)
 * data logging
 * redundant sensor safety
   * TPS faults
   * PPS faults
 * over current detection
 * over temperature detection

# Printed Circuit Board
The project files for the PCB are located in [/src/pcb/](/src/pcb/). They can be opened with [EasyEDA Pro](https://easyeda.com/). __Note that they will not open with EasyEDA Standard version__ (might convert them to std version later).

Here's a render of the printed circuit board (PCB).

![pcb_render](/doc/board_layout/pcb_v1.0.png)

The board is designed to be installed into the prototype area of the Megasquirt DIYPNP via a 20pin header connection.

![pcb_installed](/doc/board_layout/pcb_v1.0_installed.jpg)