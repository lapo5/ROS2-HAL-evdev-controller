# HAL EvDev Input Controller

HAL for an EvDev Input device ROS2 Package.

## Description

ROS2 Package to interface to any evdev input device.
The package is splitted in core and interfaces.

## EvDev Controller Configuration

Configuration file of the specific controller (output of the calibration process) must be in the install folder of evdev_teleop/conf/<controller_name>. Two files: axes_calib.json and buyttons_calib.json.

## Input/Output

Input: 

- event : name of the event (/dev/input/<event>)
- controller_name: 	to load the calibration file

Output: 

- button_subset: 	each button status will be published in button_subset/button_name (teleop_interfaces/Button)
- axis_subset: 		each axis status will be published in button_subset/axis_name (teleop_interfaces/Axis)

## Calibration

Node to calibrate the evdev controller.

## Depend

- ROS2
- EvDev
