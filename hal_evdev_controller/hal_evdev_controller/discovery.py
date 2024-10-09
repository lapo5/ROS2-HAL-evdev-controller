import evdev

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
	print("Device Path: {0}".format(device.path))
	print("Device Name: {0}".format(device.name))
	print("Device Phys: {0}".format(device.phys))


