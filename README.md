# FLIR Boson Thermal Camera ROS 2 Publisher

This package provides a ROS 2 node for capturing and publishing thermal images from the FLIR Boson camera in 8-bit format. The camera can be accessed through a Linux system using the V4L2 API, and this package uses OpenCV to handle the image data and publish it via ROS 2 topics.

## Features

- Captures thermal images from FLIR Boson camera (Boson320/Boson640).
- Supports both 16-bit raw and 8-bit YUV capture modes.
- Publishes thermal images in 8-bit format to a ROS 2 topic for integration with robotic systems.
- Simple setup for integrating FLIR Boson camera with ROS 2-based robotics platforms.

## Requirements

- FLIR Boson camera (Boson320 or Boson640).
- Linux system with V4L2 support (e.g., Ubuntu 20.04 or later).
- ROS 2 Humble or later.
- OpenCV (for image processing).
- The `v4l2` driver for Linux.
- A compatible USB or network connection for the camera.

## Installation

### Clone the repository

    cd ~/ros2_ws/src
    git clone https://github.com/MarcuSpade/boson_video_publisher.git
    cd boson_video_publisher

### Build the package

Navigate to your ROS 2 workspace:

    cd ~/ros2_ws

Install dependencies:

    rosdep install --from-paths src --ignore-src -r -y

Build the workspace:

    cd ~/ros2_ws
    colcon build --symlink-install

Source the workspace:

    source ~/ros2_ws/install/setup.bash

## Usage
### Running the Node

  Make sure your FLIR Boson camera is connected to your system via USB or network interface.
  Launch the ROS 2 node to start capturing and publishing thermal images:

    ros2 run boson_video_publisher video_publisher


This example will capture images in 8-bit AGC mode from the Boson640 camera.

## Publishing Thermal Images

The thermal images will be published on the following topic:

    /boson_video: Thermal image in 8-bit format.

You can visualize the images using rqt_image_view:

    ros2 run rqt_image_view rqt_image_view

## Troubleshooting

If the camera is not being detected, ensure the appropriate video device (/dev/video0, /dev/video1, etc.) is available and accessible.
Ensure the correct V4L2 drivers are installed for the FLIR Boson camera.
For issues with OpenCV, make sure the required version of OpenCV is installed (v4.x+ recommended).

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

This package is based on the FLIR Boson thermal camera documentation and example code provided by FLIR Systems.
Special thanks to the ROS 2 community for creating and maintaining the ROS 2 framework.


