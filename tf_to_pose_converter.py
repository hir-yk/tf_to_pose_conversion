import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
from PyKDL import Rotation


class MapToBaseLinkListener(Node):
    def __init__(self):
        super().__init__('map_to_base_link_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(PoseStamped, '/pose', 10)

        self.map_to_base_link_timer = self.create_timer(
            0.05,  # Timer period (seconds)
            self.get_base_link_position
        )

    def get_base_link_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',         # Target frame
                'base_link',   # Source frame
                rclpy.time.Duration(seconds=0),  # At the current time
                timeout=rclpy.time.Duration(seconds=1)
            )
            x, y, z = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            yaw, _, _ = Rotation.Quaternion(*quaternion).GetEulerZYX()

            self.get_logger().info(f'base_link position: x: {x:.2f} y: {y:.2f} z: {z:.2f} yaw: {yaw:.6f}')

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(pose_stamped)

        except tf2_ros.LookupException as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'Time extrapolation failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MapToBaseLinkListener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
