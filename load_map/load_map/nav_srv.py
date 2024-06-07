import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sample_navigator import BasicNavigator
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
import threading
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped

'''
please add entry point in setup.py:
entry_points={
        'console_scripts': [
            'nav_srv_node = load_map.nav_srv:main'
        ],


and add dependent in package.xml:
  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>action_msgs</exec_depend>
  <exec_depend>lifecycle_msgs</exec_depend>
'''



class NavHandler(Node):
    def __init__(self):
        super().__init__('nav_srv_node')
        # navigation init
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_poses = []
        self.goal_pose = None
        self.goal_complete = False
        # load map init
        self.cli = self.create_client(LoadMap, 'map_server/load_map')
        self.init_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()
        self.declare_parameter('map_url_param', '')

    def setPose(self, x, y, z=0.0, theta=1.0):
        print('set pose to:', x,y,z,theta)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        self.goal_pose.pose.orientation.w = theta
    
    def runNavigation(self, time_out=1000000.0):
        cmd_send = self.navigator.goToPose(self.goal_pose)
        while not self.navigator.isNavComplete() and cmd_send:
            feed_back = self.navigator.getFeedback()
            print('Distance remaining: ' + '{:.2f}'.format(feed_back.distance_remaining) + ' meters.')
            ## Detect Time out ##
            if Duration.from_msg(feed_back.navigation_time) > Duration(seconds=time_out):
                print("Navigation TIme Out")
                self.navigator.cancelNav()
        nav_result = self.navigator.getResult()
        if nav_result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded')
        elif nav_result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled')
        elif nav_result == GoalStatus.STATUS_ABORTED:
            print('Goal Failed')
        else:
            print('Goal has an invalid return status')

    def send_request(self):
        # self.req.map_url = self.get_parameter('map_url_param').get_parameter_value().string_value
        self.req.map_url = '/home/tony/map/room_2/map.yaml'      # change map url
        print('loading map:', self.req.map_url)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def pubInitialPose(self, x, y ,z = 0.0, theta = 1.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = theta
        print("Initial pose set x: {}, y: {}, w: {}".format(x, y, theta))
        self.init_pose_publisher.publish(msg)
    
    def switchMap(self):
        response = self.send_request()
        if response.result==0:
            self.pubInitialPose(0.0, 0.0)     # for initial pose
            return True
        else:
            print("Set Initial Pose Failed")
        return False

def main(args=None):
    rclpy.init(args=args)

    nav_handler = NavHandler()

    nav_handler.setPose(1.4 , -0.168, 0.0, 0.036)  # for navigation pose
    nav_handler.runNavigation()
    nav_handler.switchMap()
    nav_handler.setPose(0.8 , 0.0, 0.0, 0.036)  # for navigation pose
    nav_handler.runNavigation()

    nav_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
