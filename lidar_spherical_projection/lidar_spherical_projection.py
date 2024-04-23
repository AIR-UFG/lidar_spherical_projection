import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
from Cloud2DImageConverter import spherical_projection as sp


def projection(point_cloud, fov_up, fov_down, width, height):

    reflectance, depth, mask = sp.spherical_projection(point_cloud, fov_up, fov_down, width, height)

    reflectance = reflectance*255
    reflectance = reflectance.astype(np.uint8)

    return reflectance


class SphericalProjection(Node):
    def __init__(self):
        super().__init__('spherical_projection')

        self.declare_parameter('fov_up', 15.0)
        self.declare_parameter('fov_down', -15.0)
        self.declare_parameter('width', 2048)
        self.declare_parameter('height', 16)
        
        self.get_logger().info("Subscribed to parameters")

        # Show used parameters to user
        self.get_logger().info("fov_up: " + str(self.get_parameter('fov_up').get_parameter_value().double_value))
        self.get_logger().info("fov_down: " + str(self.get_parameter('fov_down').get_parameter_value().double_value))
        self.get_logger().info("width: " + str(self.get_parameter('width').get_parameter_value().integer_value))
        self.get_logger().info("height: " + str(self.get_parameter('height').get_parameter_value().integer_value))

        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.projection_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/lidar_spherical_projection', 10)
        self.cv_bridge = CvBridge()

    def projection_callback(self, msg):
        data = msg.data
        point_step = msg.point_step
        offset = msg.fields[0].offset

        num_points = len(data) // point_step
        points = []
        
        for i in range(num_points):
            index = i * point_step
            x, y, z, intensity = struct.unpack_from('ffff', data, offset=index+offset)
            points.append([x, y, z, intensity])

        # Convert lidar points to 2D front view image
        fov_up = self.get_parameter('fov_up').get_parameter_value().double_value
        fov_down = self.get_parameter('fov_down').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        image = projection(np.array(points), fov_up, fov_down, width, height)
    
        # Convert image to ROS Image message
        ros_image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="passthrough")

        # Publish ROS Image message
        self.publisher.publish(ros_image_msg)


def main(args=None):
    rclpy.init(args=args)

    spherical_projection = SphericalProjection()

    rclpy.spin(spherical_projection)

    spherical_projection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()