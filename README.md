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

# Building The Arduino Project
This repo uses git submodules for all required library dependencies. For this to work, you need to be using Arduino IDE version 1.6 or newer.

After cloning this repo, run `git submodule update --init --recursive` to clone the library repos located at [src/arduino/libraries/](src/arduino/libraries/). For the IDE to find the libraries directory, you need to update the sketchbook location. In the Arduino IDE, go to `File > Preferences`, and set the sketchbook location to `<path_to_this_repo>/src/arduino/`. Now you can open the project by going to `File > Sketchbook > EThrottle`.

# Printed Circuit Board
The project files for the PCB are located in [/src/pcb/](/src/pcb/). They can be opened with [EasyEDA Pro](https://easyeda.com/). __Note that they will not open with EasyEDA Standard version__ (might convert them to std version later).

Here's a render of the printed circuit board (PCB).

![pcb_render](/doc/board_layout/pcb_v1.0.png)

The board is designed to be installed into the prototype area of the Megasquirt DIYPNP via a 20pin header connection.

![pcb_installed](/doc/board_layout/pcb_v1.0_installed.jpg)