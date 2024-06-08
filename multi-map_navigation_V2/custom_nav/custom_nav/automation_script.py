import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading

class AutomatedNavigation(Node):
    def __init__(self):
        super().__init__('automated_navigation')
        #navigation init
        self.initial_pose_received = False
        self.navigation_succeeded = False

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10)
        
        self.run_sequence()

    def initialpose_callback(self, msg):
        self.initial_pose_received = True
        self.get_logger().info('Initial pose received')

    def navigation_success_callback(self):
        self.navigation_succeeded = True
        self.get_logger().info('Navigation succeeded')

    def run_sequence(self):
        self.run_command('ros2 run custom_nav client 1 0')
        self.wait_for_initial_pose()
        
        self.run_command('ros2 run custom_nav client 2 0')
        self.wait_for_navigation_success()

        self.run_command('ros2 run custom_nav client 1 1')
        self.wait_for_initial_pose()
        
        self.run_command('ros2 run custom_nav client 2 1')

    def run_command(self, command):
        self.get_logger().info(f'Running command: {command}')
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        def monitor_output():
            for line in process.stdout:
                decoded_line = line.decode('utf-8')
                print(decoded_line, end='')
                if '[controller_server]: Reached the goal!' in decoded_line or '[bt_navigator]: Navigation succeeded' in decoded_line:
                    self.navigation_success_callback()
            
        thread = threading.Thread(target=monitor_output)
        thread.start()
        process.wait()
        thread.join() 

    def wait_for_initial_pose(self):
        self.get_logger().info('Waiting for initial pose...')
        while not self.initial_pose_received:
            rclpy.spin_once(self)
        self.initial_pose_received = False

    def wait_for_navigation_success(self):
        self.get_logger().info('Waiting for navigation success...')
        while not self.navigation_succeeded:
            rclpy.spin_once(self)
        self.navigation_succeeded = False

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
