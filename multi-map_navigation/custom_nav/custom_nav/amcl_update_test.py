import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from transforms3d.euler import quat2euler, euler2quat

class AmclPoseUpdater(Node):
    def __init__(self):
        super().__init__('amcl_pose_updater')

        # Base pose values(set these values by checking amcl_pose when it's on the right position)
        self.base_pose = {
            'x': 1.73,
            'y': 0.16,
            'z': 0.0,
            'yaw': 1.0
        }

        # Subscriber to the april_tag_pose topic(camera is fix on robot,so camera_frame is actually base_link)(april_tag_pose is the AprilTag pose correspoding to camera)
        self.subscription = self.create_subscription(
            PoseStamped,
            'april_tag_pose',
            self.april_tag_pose_callback,
            10)
        
        # Publisher for the amcl_pose topic(amcl_pose update)(amcl_pose=amcl_pose + april_tag_pose)
        self.publisher_pose_with_covariance_stamped = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)

        self.get_logger().info("AmclPoseUpdater node has been started")
        self.shutdown_in_progress = False

    def calculate_new_pose(self, april_tag_pose):
        # Calculate the new position based on the base pose and april_tag_pose
        new_x = self.base_pose['x'] + april_tag_pose.pose.position.x
        new_y = self.base_pose['y'] + april_tag_pose.pose.position.y
        new_z = self.base_pose['z'] + april_tag_pose.pose.position.z

        self.get_logger().info(f"New position: x={new_x}, y={new_y}, z={new_z}")

        # Calculate the yaw from the quaternion
        april_tag_yaw = self.quaternion_to_yaw(april_tag_pose.pose.orientation)
        new_yaw = self.base_pose['yaw'] + april_tag_yaw

        self.get_logger().info(f"Base yaw: {self.base_pose['yaw']}, April tag yaw: {april_tag_yaw}, New yaw: {new_yaw}")

        # Create a new PoseWithCovarianceStamped message
        new_pose_with_covariance = PoseWithCovarianceStamped()
        new_pose_with_covariance.header.stamp = self.get_clock().now().to_msg()
        new_pose_with_covariance.header.frame_id = "map"
        
        new_pose_with_covariance.pose.pose.position.x = new_x
        new_pose_with_covariance.pose.pose.position.y = new_y
        new_pose_with_covariance.pose.pose.position.z = new_z
        
        # Convert the new yaw to a quaternion
        new_quaternion = self.yaw_to_quaternion(new_yaw)
        new_pose_with_covariance.pose.pose.orientation.x = new_quaternion[0]
        new_pose_with_covariance.pose.pose.orientation.y = new_quaternion[1]
        new_pose_with_covariance.pose.pose.orientation.z = new_quaternion[2]
        new_pose_with_covariance.pose.pose.orientation.w = new_quaternion[3]
        
        self.get_logger().info(f"New orientation: x={new_quaternion[0]}, y={new_quaternion[1]}, z={new_quaternion[2]}, w={new_quaternion[3]}")

        # Example: set zero covariance
        new_pose_with_covariance.pose.covariance = [0.0] * 36

        return new_pose_with_covariance

    def april_tag_pose_callback(self, data):
        new_pose_with_covariance = self.calculate_new_pose(data)
        
        # Publish the new pose to the amcl_pose topic
        self.publisher_pose_with_covariance_stamped.publish(new_pose_with_covariance)
        self.get_logger().info(f"Published new amcl_pose (PoseWithCovarianceStamped): {new_pose_with_covariance}")

        # Shutdown the node after publishing once
        self.get_logger().info("Shutting down the node after publishing once.")
        self.shutdown_in_progress = True

    def quaternion_to_yaw(self, quaternion):
        """Convert a quaternion into a yaw angle (in radians)."""
        euler = quat2euler([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler[2]

    def quaternion_to_yaw_from_list(self, quaternion):
        """Convert a quaternion list into a yaw angle (in radians)."""
        euler = quat2euler(quaternion)
        return euler[2]

    def yaw_to_quaternion(self, yaw):
        """Convert a yaw angle (in radians) into a quaternion."""
        q = euler2quat(0, 0, yaw)
        return q

def main(args=None):
    rclpy.init(args=args)

    amcl_pose_updater = AmclPoseUpdater()

    while rclpy.ok() and not amcl_pose_updater.shutdown_in_progress:
        rclpy.spin_once(amcl_pose_updater, timeout_sec=0.1)

    # Shutdown and clean up
    amcl_pose_updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
