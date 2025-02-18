# ROS Humble Lidar Spherical Projection

This package provides functionality to convert point cloud data from a Velodyne Lidar sensor into a 2D front view image using spherical projection techniques.

## Installation

1. Make sure you have ROS Humble installed on your system.

2. Clone this repository into your workspace:

    ```bash
    git clone https://github.com/AIR-UFG/lidar_spherical_projection.git
    ```

3. Build the package using colcon:

    ```bash
    colcon build
    ```

## Dependencies

This package depends on the following ROS 2 packages:

- `rclpy`: ROS Client Library for Python
- `sensor_msgs`: ROS messages for sensor-related data
- `cv_bridge`: ROS package to convert between ROS images and OpenCV images
- [`colorcloud`](https://github.com/AIR-UFG/colorcloud): External package used for making the spherical projections
- [`ros2_numpy`](https://github.com/Box-Robotics/ros2_numpy/tree/humble): External package used for converting ROS messages to NumPy arrays and vice versa

Make sure you have these dependencies installed before building this package.

## Usage

### Launch File

You can launch the lidar_spherical_projection node using the provided launch file. The launch file allows you to specify parameters such as field of view (FOV) up, FOV down, width, and height for the spherical projection.

To launch the node with default parameters:

```bash
ros2 launch lidar_spherical_projection lidar_spherical_projection.launch.py
```

### Parameters

To launch the node with custom parameters:

```bash
ros2 launch lidar_spherical_projection lidar_spherical_projection.launch.py fov_up:=15.0 fov_down:=-15.0 width:=440 height:=16
```

The following parameters can be configured via launch file or directly in the code:

- `fov_up`: Field of view up (default: 15.0 degrees)
- `fov_down`: Field of view down (default: -15.0 degrees)
- `width`: Width of the projection (default: 440 pixels)
- `height`: Height of the projection (default: 16 pixels)

### Topics

- Subscribed Topic:
  - `/velodyne_points`: Raw point cloud data from Velodyne Lidar sensor

- Published Topic:
  - `/lidar_spherical_projection_x`: Spherical projection image generated from the point cloud data (x-axis)
  - `/lidar_spherical_projection_y`: Spherical projection image generated from the point cloud data (y-axis)
  - `/lidar_spherical_projection_z`: Spherical projection image generated from the point cloud data (z-axis)
  - `/lidar_spherical_projection_depth`: Spherical projection image generated from the point cloud data (depth)
  - `/lidar_spherical_projection_reflectance`: Spherical projection image generated from the point cloud data (reflectance)