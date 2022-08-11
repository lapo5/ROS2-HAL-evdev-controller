#!/usr/bin/env pyhton3

##### Libraries
import json
import threading
import sys
import os

import evdev
from evdev import InputDevice, ecodes

import rclpy
from rclpy.node import Node

from teleop_interfaces.msg import AxisCmd, ButtonCmd

from std_msgs.msg import Header
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, Trigger

from ament_index_python.packages import get_package_share_directory

##### Paths to calibration files
package_share_directory = get_package_share_directory('hal_evdev_controller')
# Path to store the calibration file
PATH = package_share_directory + "/conf/"

CALIB_AXES = "axes_calib.json"
CALIB_BUTTONS = "buttons_calib.json"

##### Path to the event log
DEV_ADDR = "/dev/input/"

##### Controller class definition
class ControllerNode(Node):
    def __init__(self):
        super().__init__("evdev_controller")

        self.declare_parameter("hz", "50.0")
        self.rate = float(self.get_parameter("hz").value)

        self.declare_parameter("event", "discovery_event")
        self.event = self.get_parameter("event").value

        self.declare_parameter("controller_name", "discovery")
        self.controller_name = self.get_parameter("controller_name").value

        self.found = False

        self.declare_parameter("services.is_init", "/evdev_controller/is_init")
        self.is_init_service = self.get_parameter("services.is_init").value

        self.declare_parameter("services.stop", "/evdev_controller/stop")
        self.stop_service = self.get_parameter("services.stop").value

        self.declare_parameter("publishers.button_subset", "/teleop_cmd/button/")
        self.button_topic = self.get_parameter("publishers.button_subset").value

        self.declare_parameter("publishers.axis_subset", "/teleop_cmd/axis/")
        self.axis_topic = self.get_parameter("publishers.axis_subset").value    

        if self.event == "discovery_event":
            self.discovery_device()

        sys.stdout.flush()  # Flush screen output

        # Service: stop acquisition
        self.is_init_service = self.create_service(Trigger, self.is_init_service, self.is_init)
        
        if self.found:
            self.dev_address = DEV_ADDR + str(self.event)

            self.error = False
            self.evdev_controller_connected = True
            self.end = False

            self.is_axis = False
            self.is_button = False

            self.axis_dict = None
            self.button_dict = None

            self.actual_button = dict()
            self.actual_axis = dict()

            self.resources_path = PATH + self.controller_name + "/"

            self.get_logger().info("[HAL EvDev Controller] Calibration Folder {0}".format(self.resources_path))

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
                self.get_logger().info("[HAL EvDev Controller] Calibrate Controller {0} First!".format(self.controller_name))

            if not self.error:

                if self.is_button:
                    self.button_publishers = dict()
                    for button in self.button_dict.keys():
                        name = self.button_dict[button][0]
                        self.button_publishers[button] = self.create_publisher(ButtonCmd, self.button_topic+name, 1)

                if self.is_axis:
                    self.axis_publishers = dict()
                    for axis in self.axis_dict.keys():
                        name = self.axis_dict[axis][0]
                        self.axis_publishers[axis] = self.create_publisher(AxisCmd, self.axis_topic+name, 1)

                self.declare_parameter("publishers.heartbeat", "/evdev_controller/heartbeat")
                self.heartbeat_topic_name = self.get_parameter("publishers.heartbeat").value
                self.heartbeat_publisher = self.create_publisher(EmptyMsg, self.heartbeat_topic_name, 1)
                
                self.thread1 = threading.Thread(target=self.update_cmds, daemon=True)
                
                self.stop_service = self.create_service(Empty, self.stop_service, self.stop)

                # Start of the commands update thread
                self.initialize_cmds()
                self.thread1.start()

                self.axis_timer = self.create_timer(1.0/self.rate, self.publish_commands) #50 Hz
                self.get_logger().info("[HAL EvDev Controller] Node Ready!")

        else:
            self.get_logger().info("[HAL EvDev Controller] No EvDev Controller found, check connection!")


    def discovery_device(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if device.name == "Mad Catz Saitek Side Panel Control Deck":
                self.get_logger().info("Found LogitechPanel on path: {0}".format(device.path))
                self.event = device.path.split('/')[-1]
                self.controller_name = "logitech_panel"
                self.found = True

            elif device.name == "Sony Computer Entertainment Wireless Controller"  or device.name == "Wireless Controller":
                self.get_logger().info("Found PS4 Joystick on path: {0}".format(device.path))
                self.event = device.path.split('/')[-1]
                self.controller_name = "ps4_joystick"
                self.found = True

            elif device.name == "Microsoft SideWinder Precision 2 Joystick":
                self.get_logger().info("Found Joystick Slide Winder on path: {0}".format(device.path))
                self.event = device.path.split('/')[-1]
                self.controller_name = "slide_winder"
                self.found = True

            elif device.name == "Logitech Logitech Cordless RumblePad 2":
                self.get_logger().info("Found Logitech Wireless Joystick on path: {0}".format(device.path))
                self.event = device.path.split('/')[-1]
                self.controller_name = "logitech_rumblepad"
                self.found = True

            elif device.name == "Logitech Gamepad F710":
                self.get_logger().info("Found Logitech Gamepad F710 on path: {0}".format(device.path))
                self.event = device.path.split('/')[-1]
                self.controller_name = "logitech_gamepad_f710"
                self.found = True

            else:
                self.get_logger().info("Found unknown device on path: {0}".format(device.path))
                self.get_logger().info("Name: {0}".format(device.name))
                self.event = device.path.split('/')[-1]
                self.controller_name = input("Insert Controller Name (must match controller name on conf folder): ")
                self.found = True


    def stop(self, request, response):

        self.end = True
        return response


    def is_init(self, request, response):
        
        response.success = self.found
        return response


    def initialize_cmds(self):
        
        if self.is_axis:
            self.actual_axis = self.axis_dict.copy()
            for key in self.axis_dict.keys():
                self.actual_axis[key] = self.axis_dict[key][1][2]

        if self.is_button:
            self.actual_button = self.button_dict.copy()
            for key in self.button_dict.keys():
                self.actual_button[key] = 0


    def update_cmds(self):
        while(True):
            
            while(not self.evdev_controller_connected):
                self.found = False
                self.discovery_device()
                if self.found:
                    self.evdev_controller_connected = True
                    self.get_logger().info("Device Re-Connected!")
            
            while not self.error and not self.end and self.evdev_controller_connected:
                try:
                    device = InputDevice(self.dev_address)

                    for event in device.read_loop():
                        if self.is_axis and (str(event.code) in self.axis_dict.keys()) and (event.type != ecodes.SYN_REPORT) and (event.type != 4):
                            self.actual_axis[str(event.code)] = event.value 
                        elif self.is_button and str(event.code) in self.button_dict.keys():
                            self.actual_button[str(event.code)] = event.value
                        else:
                            continue

                except:
                    self.get_logger().info("Device Disconnected!")
                    self.evdev_controller_connected = False


    def publish_commands(self):

        if not self.end and self.evdev_controller_connected:
            for key in self.actual_axis.keys():
                msg = AxisCmd()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "Axis_" +  str(self.axis_dict[key][0]) + "_Cmd"
                msg.axis_cmd = self.actual_axis[key]
                msg.min_value = self.axis_dict[key][1][0]
                msg.max_value = self.axis_dict[key][1][1]
                msg.steady_value = self.axis_dict[key][1][2]
                self.axis_publishers[key].publish(msg)
                
            for key in self.actual_button.keys():
                msg = ButtonCmd()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "Button_" +  str(self.button_dict[key][0]) + "_Cmd"
                msg.button_cmd = self.actual_button[key]
                self.button_publishers[key].publish(msg)


            heartbeat_msg = EmptyMsg()
            self.heartbeat_publisher.publish(heartbeat_msg)


    def exit(self):
        self.end = True
        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[EvDev Controller] Node stopped cleanly')
        node.exit()
    except BaseException:
        node.get_logger().info('[EvDev Controller] Exception:', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown() 



if __name__ == "__main__":
    main()
