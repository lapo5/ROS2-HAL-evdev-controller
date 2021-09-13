import evdev
from evdev import InputDevice, ecodes

##### Path to the event log
DEV_ADDR = "/dev/input/"

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
	print("Found device - \nPath: {0}\nName: {1}\nPhys: {2}\n\n".format(device.path, device.name, device.phys))

	if device.name == "Mad Catz Saitek Side Panel Control Deck":
		print("Found Panel on path: {0}".format(device.path))
		event = device.path.split('/')[-1]

	if device.name == "Mad Catz Saitek Heavy Eqpt. Wheel & Pedal":
		print("Found Wheel/Pedal on path: {0}".format(device.path))
		event = device.path.split('/')[-1]

	if device.name == "Sony Computer Entertainment Wireless Controller":
		print("Found PS4 Joystick on path: {0}".format(device.path))
		event = device.path.split('/')[-1]

dev_address = DEV_ADDR + str(event)

while True:
	device = InputDevice(dev_address)
	for event in device.read_loop():
		if int(event.type) == 3 or int(event.type) == 1:
			print("Captured event - Code: {0}, Type: {1}, Value: {2}".format(event.code, event.type, event.value))
		