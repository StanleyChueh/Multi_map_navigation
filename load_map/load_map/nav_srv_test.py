import cv2
import numpy as np
from apriltag import Detector
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from sample_navigator import BasicNavigator

class NavHandler(Node):
    def __init__(self):
        super().__init__('nav_srv_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_poses = []
        self.goal_pose = None
        self.goal_complete = False
        self.detector = Detector()
        self.tag_centered = False
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Load map service init
        self.cli = self.create_client(LoadMap, 'map_server/load_map')
        self.init_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = LoadMap.Request()

    def calculate_center(self, corners):
        return tuple(map(int, np.mean(corners, axis=0)))

    def april_tag_callback(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        twist = Twist()
        for detection in detections:
            center = self.calculate_center(detection.corners)
            if abs(center[0] - frame.shape[1] // 2) <= 10:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.tag_centered = True
                return

            if center[0] < frame.shape[1] // 2:
                print("Turning Left!")
            else:
                print("Turning Right!")

            twist.angular.z = np.clip(0.5 * np.sign(center[0] - frame.shape[1] // 2), -0.5, 0.5)
            self.cmd_vel_pub.publish(twist)
            return

    def run(self):
        cap = cv2.VideoCapture('/dev/video0')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                break
            self.april_tag_callback(frame)
            if self.tag_centered:
                print("AprilTag is centered. Shutting down.")
                break

        cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

    def calibrate_apriltag(self):
        # Assuming AprilTag calibration code is in this method
        pass

    def switch_map(self):
        response = self.send_request()
        if response.result == 0:
            self.pub_initial_pose(0.0, 0.0)  # Set initial pose
            return True
        else:
            print("Set Initial Pose Failed")
        return False

    def send_request(self):
        self.req.map_url = '/home/tony/map/room_2/map.yaml'  # Change map URL
        print('Loading map:', self.req.map_url)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def pub_initial_pose(self, x, y, z=0.0, theta=1.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = theta
        print("Initial pose set x: {}, y: {}, w: {}".format(x, y, theta))
        self.init_pose_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nav_handler = NavHandler()

    nav_handler.set_pose(1.4, -0.168, 0.0, 0.036)  # Set navigation pose
    nav_handler.run_navigation()
    
    # Calibration
    nav_handler.calibrate_apriltag()

    # Continue with navigation
    nav_handler.switch_map()
    nav_handler.set_pose(0.8, 0.0, 0.0, 0.036)  # Set navigation pose
    nav_handler.run_navigation()

    nav_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
