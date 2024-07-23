import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
from colorcloud.behley2019iccv import SphericalProjection, ProjectionTransform
from ros2_numpy.point_cloud2 import pointcloud2_to_array
from numpy.lib.recfunctions import structured_to_unstructured


class SphericalProjectionRos(Node):
    def __init__(self):
        super().__init__('spherical_projection')

        self.declare_parameter('fov_up', 15.0)
        self.declare_parameter('fov_down', -15.0)
        self.declare_parameter('width', 440)
        self.declare_parameter('height', 16)
        
        fov_up = self.get_parameter('fov_up').get_parameter_value().double_value
        fov_down = self.get_parameter('fov_down').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        self.fields = ['x', 'y', 'z', 'intensity']

        self.spherical_projection = SphericalProjection(fov_up, fov_down, width, height)
        self.projection_transform = ProjectionTransform(self.spherical_projection)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.projection_callback,
            10
        )
        self.publisher_x = self.create_publisher(Image, '/lidar_spherical_projection_x', 10)
        self.publisher_y = self.create_publisher(Image, '/lidar_spherical_projection_y', 10)
        self.publisher_z = self.create_publisher(Image, '/lidar_spherical_projection_z', 10)
        self.publisher_depth = self.create_publisher(Image, '/lidar_spherical_projection_depth', 10)
        self.publisher_reflectance = self.create_publisher(Image, '/lidar_spherical_projection_reflectance', 10)

        self.cv_bridge = CvBridge()

    def projection_callback(self, msg):
        
        pointcloud = structured_to_unstructured(pointcloud2_to_array(msg)[self.fields])
        pointcloud[:, 3] = pointcloud[:, 3] / 255 # Normalize the intensity values

        frame_img, _, _ = self.projection_transform(pointcloud, None, None) # Get the spherical projection
        frame_img = (frame_img * 255).astype(np.uint8) # Multiply the image by 255 to get the correct range

        # Extract the components of the image
        # Convert the image to a ROS message
        # Publish the ROS message

        x_img = frame_img[:, :, 0] 
        ros_x_msg = self.cv_bridge.cv2_to_imgmsg(x_img, encoding='passthrough') 
        self.publisher_x.publish(ros_x_msg) 

        y_img = frame_img[:, :, 1]
        ros_y_msg = self.cv_bridge.cv2_to_imgmsg(y_img, encoding='passthrough')
        self.publisher_y.publish(ros_y_msg)

        z_img = frame_img[:, :, 2]
        ros_z_msg = self.cv_bridge.cv2_to_imgmsg(z_img, encoding='passthrough')
        self.publisher_z.publish(ros_z_msg)

        depth_img = frame_img[:, :, 3]
        ros_depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_img, encoding='passthrough')
        self.publisher_depth.publish(ros_depth_msg)

        reflectance_img = frame_img[:, :, 4]
        ros_reflectance_msg = self.cv_bridge.cv2_to_imgmsg(reflectance_img, encoding='passthrough')
        self.publisher_reflectance.publish(ros_reflectance_msg)


def main(args=None):
    rclpy.init(args=args)

    spherical_projection = SphericalProjectionRos()

    rclpy.spin(spherical_projection)

    spherical_projection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()