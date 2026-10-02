from setuptools import find_packages, setup

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahdi',
    maintainer_email='mhachicha01@gmail.com',
    description='Serial bridge between ROS 2 and the ESP32 low-level controller',
    license='TBD',
    entry_points={
        'console_scripts': [
            'esp32_bridge = robot_bridge.esp32_bridge:main',
        ],
    },
)
