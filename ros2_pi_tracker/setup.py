from setuptools import setup

package_name = 'ros2_pi_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'opencv-python', 'ultralytics', 'rclpy'],
    zip_safe=True,
    maintainer='SaulP1',
    maintainer_email='your_email@example.com',
    description='ROS2 wrapper for Pi Tracker functionality using YOLO model.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = ros2_pi_tracker.tracker_node:main',
        ],
    },
)