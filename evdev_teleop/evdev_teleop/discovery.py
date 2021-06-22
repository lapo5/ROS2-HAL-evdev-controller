import evdev

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
	print(device.path, device.name, device.phys)

	if device.name == "Mad Catz Saitek Side Panel Control Deck":
		print("Found on path: {0}".format(device.path))

