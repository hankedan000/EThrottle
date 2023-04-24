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
Here's a photo of the printed circuit board (PCB) that I've designed.

![PCB_v1](/doc/board_layout/pcb_v1.png)

The [EasyEDA](https://easyeda.com/) project files for the board are located in [/src/pcb/](/src/pcb/). My board is designed to be installed into the prototype area of the Megasquirt DIYPNP via a 20pin header connection.
