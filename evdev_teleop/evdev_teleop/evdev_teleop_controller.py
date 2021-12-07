#!/usr/bin/env pyhton3

##### Libraries
import rclpy
from rclpy.node import Node
import evdev
from evdev import InputDevice, ecodes
import json
import threading
import sys

##### Interfaces
from teleop_interfaces.msg import AxisCmd, ButtonCmd

from std_msgs.msg import Header
from std_srvs.srv import Empty, Trigger

from ament_index_python.packages import get_package_share_directory
##### Paths to calibration files
package_share_directory = get_package_share_directory('evdev_teleop')
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

            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            for device in devices:
                if device.name == "Mad Catz Saitek Side Panel Control Deck":
                    print("Found LogitechPanel on path: {0}".format(device.path))
                    self.event = device.path.split('/')[-1]
                    self.controller_name = "logitech_panel"
                    self.found = True

                if device.name == "Sony Computer Entertainment Wireless Controller"  or device.name == "Wireless Controller":
                    print("Found PS4 Joystick on path: {0}".format(device.path))
                    self.event = device.path.split('/')[-1]
                    self.controller_name = "ps4_joystick"
                    self.found = True

                if device.name == "Microsoft SideWinder Precision 2 Joystick":
                    print("Found Joystick Slide Winder on path: {0}".format(device.path))
                    self.event = device.path.split('/')[-1]
                    self.controller_name = "slide_winder"
                    self.found = True

        sys.stdout.flush()  # Flush screen output

        # Service: stop acquisition
        self.is_init_service = self.create_service(Trigger, self.is_init_service, self.is_init)
        
        if self.found:
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
                print("Calibrate Controller {0} First!".format(self.controller_name))

            if not self.error:
                # Definition of the publisher functions
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
                

                self.thread1 = threading.Thread(target=self.update_cmds, daemon=True)
                
                self.stop_service = self.create_service(Empty, self.stop_service, self.stop)

                # Start of the commands update thread
                self.initialize_cmds()
                self.thread1.start()

                # Timers for both axis and buttons publishers
                self.axis_timer = self.create_timer(1.0/self.rate, self.publish_commands) #50 Hz
                self.get_logger().info("[EvDev Controller] Node Ready!")
        else:
            self.get_logger().info("[EvDev Controller] No EvDev Controller found, check connection!")


    def stop(self, request, response):
        self.end = True

        return response


    def is_init(self, request, response):
        
        response.success = self.found

        return response


    def initialize_cmds(self):
        # Initialize axis cmd at the middle value of their excursion
        if self.is_axis:
            self.actual_axis = self.axis_dict.copy()
            for key in self.axis_dict.keys():
                self.actual_axis[key] = self.axis_dict[key][1][2]

        # Initialize button cmd at 0
        if self.is_button:
            self.actual_button = self.button_dict.copy()
            for key in self.button_dict.keys():
                self.actual_button[key] = 0


    def update_cmds(self):
        while not self.error and not self.end:
            device = InputDevice(self.dev_address)
            for event in device.read_loop():
                if self.is_axis and (str(event.code) in self.axis_dict.keys()) and (event.type != ecodes.SYN_REPORT) and (event.type != 4):
                    self.actual_axis[str(event.code)] = event.value 
                elif self.is_button and str(event.code) in self.button_dict.keys():
                    self.actual_button[str(event.code)] = event.value
                else:
                    continue


    def publish_commands(self):
        if not self.end:
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


    def exit(self):
        self.end = True
        

##### Main function to loop
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



##### Main Loop
if __name__ == "__main__":
    main()
