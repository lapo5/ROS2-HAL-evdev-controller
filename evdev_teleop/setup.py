import os
from glob import glob
from setuptools import setup

package_name = 'evdev_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'conf', 'logitech_panel'), glob("conf/logitech_panel/*")),
        (os.path.join('share', package_name, 'conf', 'logitech_wheel'), glob("conf/logitech_wheel/*")),
        (os.path.join('share', package_name, 'conf', 'ps4_joystick'), glob("conf/ps4_joystick/*")),
        (os.path.join('share', package_name, 'conf', 'slide_winder'), glob("conf/slide_winder/*")),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ale_scar',
    maintainer_email='ale_scar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "controller = evdev_teleop.evdev_teleop_controller:main",
        "calibrate_controller = evdev_teleop.calibrate_evdev_teleop_controller:main",
        ],
    },
)
