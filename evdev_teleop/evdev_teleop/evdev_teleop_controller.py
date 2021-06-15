#!/usr/bin/env pyhton3

##### Libraries
import rclpy
from rclpy.node import Node
from evdev import InputDevice, ecodes
import json
import os
import threading
import shutil
import time
import sys

##### Interfaces
from teleop_interfaces.msg import AxisCmd, ButtonCmd
from std_msgs.msg import Header

##### Paths to calibration files
PATH = "./src/Teleoperation_Module/evdev_teleop/conf/"
CALIB_AXES = "axes_calib.json"
CALIB_BUTTONS = "buttons_calib.json"

##### Path to the event log
DEV_ADDR = "/dev/input/"



##### Controller class definition
class ControllerNode(Node):
	def __init__(self):
		super().__init__("evdev_controller_node")

		self.get_logger().info("Controller node is awake...")

		self.declare_parameter("event", "event2")
		self.declare_parameter("controller_name", "logitech_panel")

		self.controller_name = self.get_parameter("controller_name").value
		self.event = self.get_parameter("event").value

		self.dev_address = DEV_ADDR + str(self.event)

		self.error = False
		self.end = False

		self.is_axis = False
		self.is_button = False

		self.axis_dict = None
		self.button_dict = None

		self.actual_button = dict()
		self.actual_axis = dict()

		self.resources_path = PATH + self.controller_name + "/"
		print(self.resources_path)

		self.thread1 = threading.Thread(target=self.update_cmds, daemon=True)

		# Upload calibration data
		try:
			with open(self.resources_path+CALIB_AXES, "r") as readfile:
				self.axis_dict = json.load(readfile)
			with open(self.resources_path+CALIB_BUTTONS, "r") as readfile:
				self.button_dict = json.load(readfile)

			if self.axis_dict != None: self.is_axis = True
			if self.button_dict != None: self.is_button = True

		except:
			self.error = True
			print("Calibrate Controller First!")

		if not self.error:
			# Definition of the publisher functions
			if self.is_button:
				self.button_publishers = dict()
				for button in self.button_dict.keys():
					name = self.button_dict[button][0]
					self.button_publishers[button] = self.create_publisher(ButtonCmd, "teleop_cmd/button/"+name, 10)

			if self.is_axis:
				self.axis_publishers = dict()
				for axis in self.axis_dict.keys():
					name = self.axis_dict[axis][0]
					self.axis_publishers[axis] = self.create_publisher(AxisCmd, "teleop_cmd/axis/"+name, 10)
			
			# Start of the commands update thread
			self.initialize_cmds()
			self.thread1.start()

			# Timers for both axis and buttons publishers
			self.axis_timer = self.create_timer(0.1, self.publish_axis)
			self.button_timer = self.create_timer(0.1, self.publish_button)

	

	### This function remaps the axis and buttons and sets their initial value
	def initialize_cmds(self):
		# Initialize axis cmd at the middle value of their excursion
		if self.is_axis:
			self.actual_axis = self.axis_dict.copy()
			for key in self.axis_dict.keys():
				self.actual_axis[key] = self.axis_dict[key][1][1] // 2

		# Initialize button cmd at 0
		if self.is_button:
			self.actual_button = self.button_dict.copy()
			for key in self.button_dict.keys():
				self.actual_button[key] = 0



	### This function update the axis/button cmds in a background thread
	def update_cmds(self):
		while not self.error and not self.end:
			device = InputDevice(self.dev_address)
			for event in device.read_loop():
				if self.is_axis and (str(event.code) in self.axis_dict.keys()) and (event.type != ecodes.SYN_REPORT):
					self.actual_axis[str(event.code)] = event.value 
				elif self.is_button and str(event.code) in self.button_dict.keys():
					self.actual_button[str(event.code)] = event.value
				else:
					continue


	def publish_axis(self):
		for key in self.actual_axis.keys():
			msg = AxisCmd()
			now = time.time()
			msg.header = Header()
			msg.header.stamp.sec = int(now)
			msg.header.stamp.nanosec = int(now* 1e9) % 1000000000
			msg.header.frame_id = "Axis_" +  str(self.axis_dict[key][0]) + "_Cmd"
			msg.axis_cmd = self.actual_axis[key]
			self.axis_publishers[key].publish(msg)

	def publish_button(self):
		for key in self.actual_button.keys():
			msg = ButtonCmd()
			now = time.time()
			msg.header = Header()
			msg.header.stamp.sec = int(now)
			msg.header.stamp.nanosec = int(now* 1e9) % 1000000000
			msg.header.frame_id = "Button_" +  str(self.button_dict[key][0]) + "_Cmd"
			msg.button_cmd = self.actual_button[key]
			self.button_publishers[key].publish(msg)

	def exit(self):
		self.end = True
		

##### Main function to loop
def main(args=None):
	rclpy.init(args=args)
	node = ControllerNode()
	
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('Node stopped cleanly')
		node.exit()
	except BaseException:
		print('exception in node:', file=sys.stderr)
		raise
	finally:
		# Destroy the node explicitly
		# (optional - Done automatically when node is garbage collected)
		node.destroy_node()
		rclpy.shutdown() 



##### Main Loop
if __name__ == "__main__":
	main()
