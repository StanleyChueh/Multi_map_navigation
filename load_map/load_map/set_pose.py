import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class SetPose(Node):
    def __init__(self):
        super().__init__('set_pose_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    def sent_pose(self):
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.pose.position.x = 0.0
        init_pose.pose.pose.position.y = 0.0
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = 0.0
        init_pose.pose.pose.orientation.w = 1.0
        # init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
        self.publisher_.publish(init_pose)
        self.get_logger().info('Publishing')

def main(args=None):
    rclpy.init(args=args)

    set_pose_publisher = SetPose()

    set_pose_publisher.sent_pose()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    set_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
