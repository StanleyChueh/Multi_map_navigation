import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sample_navigator import BasicNavigator
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
import threading

class NavHandler(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.goal_poses = []
        self.goal_pose = None
        self.goal_complete = False

    def setPose(self, x, y, z=0.0, theta=1.0):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = '/map'
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
        

 



def main(args=None):
    rclpy.init(args=args)

    nav_handler = NavHandler()

    nav_handler.setPose(1.4, -0.168, 0.0, 1.0)
    nav_handler.runNavigation()

    nav_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
