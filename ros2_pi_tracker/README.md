# ROS2 Pi Tracker

This ROS2 package provides a wrapper around the existing Pi Tracker functionality implemented in `pi_tracker.py`. The package integrates object tracking capabilities using a camera and the YOLO model within the ROS2 framework.

## Overview

The Pi Tracker utilizes a camera to detect and track objects in real-time. It leverages the YOLO (You Only Look Once) model for object detection and provides logging functionality for tracking data.

## Installation

To install the ROS2 Pi Tracker package, follow these steps:

1. Clone the repository:
   ```
   git clone https://github.com/SaulP1/Drone-YoloV8Nano.git
   cd Drone-YoloV8Nano/ros2_pi_tracker
   ```

2. Install dependencies:
   Ensure you have ROS2 installed and set up. Then, install the required Python packages:
   ```
   pip install -r requirements.txt
   ```

3. Build the package:
   ```
   colcon build
   ```

4. Source the setup file:
   ```
   source install/setup.bash
   ```

## Usage

To run the Pi Tracker node, use the following command:
```
ros2 launch ros2_pi_tracker pi_tracker.launch.py
```

This will start the tracking node and any other necessary nodes as defined in the launch file.

## Files

- `pi_tracker.py`: Contains the main tracking functionality.
- `ros2_pi_tracker/ros2_pi_tracker/__init__.py`: Marks the directory as a Python package.
- `ros2_pi_tracker/ros2_pi_tracker/pi_tracker_adapter.py`: Adapts the Pi Tracker functionality for ROS2 integration.
- `ros2_pi_tracker/ros2_pi_tracker/tracker_node.py`: Defines the ROS2 node for tracking.
- `ros2_pi_tracker/launch/pi_tracker.launch.py`: Launch file for starting the tracking node.
- `ros2_pi_tracker/package.xml`: Package metadata and dependencies.
- `ros2_pi_tracker/setup.py`: Setup script for the ROS2 package.
- `ros2_pi_tracker/setup.cfg`: Configuration settings for the package.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.