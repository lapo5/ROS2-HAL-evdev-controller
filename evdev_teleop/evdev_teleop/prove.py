import evdev

import evdev

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
	print(device.path, device.name, device.phys)

device = evdev.InputDevice('/dev/input/event3')
print(device)


for event in device.read_loop():
	print(evdev.categorize(event))
	print(event.code)
	print(event.value)
