#!/usr/bin/env pyhton3

##### Libraries
import rclpy
from rclpy.node import Node
from evdev import InputDevice, ecodes
import json
import os
import threading
import shutil
import statistics
import evdev
from evdev import InputDevice, ecodes
import sys
import math

##### Interfaces
from teleop_interfaces.msg import AxisCmd, ButtonCmd

##### Paths to calibration files
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('evdev_teleop')

PATH = package_share_directory + "/conf/"

CALIB_AXES = "axes_calib.json"
CALIB_BUTTONS = "buttons_calib.json"

DEV_ADDR = "/dev/input/"


class CalibrateControllerNode(Node):
	def __init__(self):
		super().__init__("calibrate_controller")

		self.declare_parameter("event", "discovery_event")
		self.declare_parameter("controller_name", "None")

		self.controller_name = self.get_parameter("controller_name").value
		self.event = self.get_parameter("event").value

		self.dev_address = DEV_ADDR + str(self.event)

		self.stop_calib = False

		self.is_axis = False
		self.is_button = False

		self.axis_dict = None
		self.button_dict = None

		self.actual_button = dict()
		self.actual_axis = dict()

		self.found = False

		if self.event == "discovery_event":
			
			devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
			for device in devices:
				if device.name == "Mad Catz Saitek Side Panel Control Deck":
					self.get_logger().info("Found LogitechPanel on path: {0}".format(device.path))
					self.event = device.path.split('/')[-1]
					self.controller_name = "logitech_panel"
					self.found = True

				if device.name == "Sony Computer Entertainment Wireless Controller"  or device.name == "Wireless Controller":
					self.get_logger().info("Found PS4 Joystick on path: {0}".format(device.path))
					self.event = device.path.split('/')[-1]
					self.controller_name = "ps4_joystick"
					self.found = True

				if device.name == "Microsoft SideWinder Precision 2 Joystick":
					self.get_logger().info("Found Joystick Slide Winder on path: {0}".format(device.path))
					self.event = device.path.split('/')[-1]
					self.controller_name = "slide_winder"
					self.found = True

		sys.stdout.flush()  # Flush screen output

		self.dev_address = DEV_ADDR + str(self.event)

		self.resources_path = PATH + self.controller_name + "/"
		self.get_logger().info(self.resources_path)

		# Upload calibration data
		while not self.stop_calib:
			key = input("Calibrate the axes? [y/else]:\t")
			
			# Fast check after input
			if key == "Y" or key == 'y':
				self.get_logger().info("Calibration of the axes started...")
				self.axis_dict = self.calibrate_controller_axes(300)
				self.get_logger().info("Calibration is over...")
				self.is_axis = True

			key = input("Calibrate the buttons? [y/else]:\t")
			
			# Fast check after input
			if key == "Y" or key == 'y':
				self.get_logger().info("Calibration of the buttons started...")
				self.button_dict = self.calibrate_controller_buttons(10)
				self.get_logger().info("Calibration is over...")
				self.is_button = True

			self.stop_calib = self.show_calib_result(self.is_axis, self.is_button)
			if self.stop_calib:
				break


	def calibrate_controller_axes(self, size_th):

		# Initialization of some variables
		key = None
		cmd_candidates = dict()
		cmd_dict = dict()

		while(key != "Q"):
			key = input("Type the name of the command to be registered or press Q to terminate.\n")
			
			# Fast check after input
			if key == "Q" or key == 'q':
				break

			# Iterate all the registered events
			
			input("Press Enter and Start Moving Axis")

			device = InputDevice(self.dev_address)
			for event in device.read_loop():

				# Discard the syncronization event
				if event.type != ecodes.SYN_REPORT and event.code != 4:
					
					# Initialize the event if never happend so far
					if event.code not in cmd_candidates.keys():
						cmd_candidates[event.code] = []
					
					# Save the event value in a list
					cmd_candidates[event.code].append(event.value)

					# If enough values have been acquired break the loop
					ret, cmd_code = self.check_subject_cmd(cmd_candidates, size_th)

					if ret:
						self.get_logger().info("cmd_code: {0}".format(cmd_code))
						break

			input("Press Enter and Move Axis to Minimum and Maximum")
			values = []
			for event in device.read_loop():

				# Discard the syncronization event
				if event.type != ecodes.SYN_REPORT and event.code != 4:

					# Save the event value in a list
					if event.code == cmd_code:
						values.append(event.value)

					# If enough values have been acquired break the loop
					ret, minimum_value, maximum_value = self.check_minmax_cmd(values, size_th)

					if ret:		
						self.get_logger().info(minimum_value)	
						self.get_logger().info(maximum_value)		
						break

			steady_value = input("Enter Steady Value the Axis")

			try:
				steady_value = int(steady_value)
			except:
				steady_value = math.ceil((maximum_value - minimum_value) / 2)

			cmd_dict[cmd_code] = [key, [minimum_value, maximum_value, steady_value]]

			# Reset the local dictionary
			cmd_candidates = dict()

		self.get_logger().info(cmd_dict)
		return cmd_dict

	### This function defines a dictionary with command name and range of values
	def calibrate_controller_buttons(self, size_th):

		# Initialization of some variables
		key = None
		cmd_candidates = dict()
		cmd_dict = dict()

		while(key != "Q"):
			key = input("Type the name of the command to be registered or press Q to terminate.\n")
			
			# Fast check after input
			if key == "Q" or key == 'q':
				break

			# Iterate all the registered events
			input("Press Enter and Start Moving Button")

			device = InputDevice(self.dev_address)
			for event in device.read_loop():

				# Discard the syncronization event
				if event.type != ecodes.SYN_REPORT and event.code != 4:
					
					# Initialize the event if never happend so far
					if event.code not in cmd_candidates.keys():
						cmd_candidates[event.code] = []
					
					# Save the event value in a list
					cmd_candidates[event.code].append(event.value)

					# If enough values have been acquired break the loop
					ret, cmd_code = self.check_subject_cmd(cmd_candidates, size_th)

					if ret:
						self.get_logger().info("cmd_code: {0}".format(cmd_code))
						break

			input("Press Enter and Move Button Again")
			values = []
			for event in device.read_loop():

				# Discard the syncronization event
				if event.type != ecodes.SYN_REPORT and event.code != 4:

					
					# Save the event value in a list
					if event.code == cmd_code:
						values.append(event.value)

					# If enough values have been acquired break the loop
					ret, minimum_value, maximum_value = self.check_minmax_cmd(values, size_th)

					if ret:		
						self.get_logger().info(minimum_value)	
						self.get_logger().info(maximum_value)		
						break

			cmd_dict[cmd_code] = [key, [minimum_value, maximum_value]]

			# Reset the local dictionary
			cmd_candidates = dict()

		self.get_logger().info(cmd_dict)
		return cmd_dict


	### This function check the subject command to be stored
	def check_subject_cmd(self, candidates, lim_size):

		best_candidate_len = 0
		is_enough = False
		cmd_code = None

		for cmd in candidates.keys():

			# Solve a maximum problem with a threshold given by lim_size
			if len(candidates[cmd]) >= lim_size:
				if len(candidates[cmd]) > best_candidate_len:

					# Do not count axes if we are calibrating buttons 
					if not((lim_size == 10) and (sum(candidates[cmd]) > len(candidates[cmd]))):
						best_candidate_len = len(candidates[cmd])
						is_enough = True
						cmd_code = cmd

		return is_enough, cmd_code
	

	def check_minmax_cmd(self, candidates,lim_size):
		is_enough = False
		minumum = None
		maximum = None

		# Solve a maximum problem with a threshold given by lim_size
		if len(candidates) >= lim_size:
			
			is_enough = True
			minumum = min(candidates)
			maximum = max(candidates)

		return is_enough, minumum, maximum
	
	
	def check_steady_value_cmd(self, candidates,lim_size):

		is_enough = False
		steady_value = None

		# Solve a maximum problem with a threshold given by lim_size
		if len(candidates) >= lim_size:
			
			is_enough = True
			steady_value = statistics.median(candidates)

		return is_enough, steady_value
	

	def check_maximum_cmd(self, candidates,lim_size):

		is_enough = False
		maximum = None

		# Solve a maximum problem with a threshold given by lim_size
		if len(candidates) >= lim_size:
			
			is_enough = True
			maximum = max(candidates)

		return is_enough, maximum
	

	def show_calib_result(self, is_axis, is_button):

		# Show the axes legend
		if is_axis:
			self.get_logger().info("\n\n =========== AXIS CMD LEGEND ==========\n")
			for axis in self.axis_dict.keys():
				self.get_logger().info(f"Name: {self.axis_dict[axis][0]}\tCode: {axis}\tRange: {self.axis_dict[axis][1]}\n")

		# Show the buttons legend
		if is_button:
			self.get_logger().info("\n\n ========== BUTTONS CMD LEGEND =========\n")
			for button in self.button_dict.keys():
				self.get_logger().info(f"Name: {self.button_dict[button][0]}\tCode: {button}\tRange: {self.button_dict[button][1]}\n")

		# If calibration is successful, let the user store it as a JSON file
		if input("Do you want to save this calibration? [y/else]:\t") == "y":

			# If an axes calibration is available, store it in axes_calib.json
			self.get_logger().info(self.resources_path)
			if os.path.exists(self.resources_path):
				shutil.rmtree(self.resources_path)
			os.mkdir(self.resources_path)
			if is_axis:
				if os.path.exists(self.resources_path+CALIB_AXES):
					os.remove(self.resources_path+CALIB_AXES)
				with open(self.resources_path+CALIB_AXES, "w+") as outfile:
					json.dump(self.axis_dict, outfile)

			# If a buttons calibration is available, store it in buttons_calib.json
			if is_button:
				if os.path.exists(self.resources_path+CALIB_BUTTONS):
					os.remove(self.resources_path+CALIB_BUTTONS)
				with open(self.resources_path+CALIB_BUTTONS, "w+") as outfile:
					json.dump(self.button_dict, outfile)

			self.get_logger().info("Calibration data have been saved successfully.")
			return True

		else:

			self.get_logger().info("Calibration data have been rejected. Try again.")
			return False

		


def main(args=None):
	rclpy.init(args=args)
	node = CalibrateControllerNode()
	
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info('[EvDev Calibration] Node Stopped Cleanly')
	except BaseException:
		node.get_logger().info('[EvDev Calibration] Exception:', file=sys.stderr)
		raise
	finally:
		rclpy.shutdown() 



##### Main Loop
if __name__ == "__main__":
	main()
