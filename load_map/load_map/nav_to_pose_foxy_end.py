import rclpy 
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavActionServer():
    def __init__(self):
        rclpy.init()

        self.nav2_node = rclpy.create_node('movebase_client')
        self.action_client = ActionClient(self.nav2_node, NavigateToPose, 'navigate_to_pose')

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            print("Action server not available!")
            self.nav2_node.destroy_node()
            rclpy.shutdown()
            return
        
        self.goal_poses = []
        
    def setGoalPose(self, goal_x, goal_y, goal_theta):
        goal_pose_msg = NavigateToPose.Goal()
        goal_pose_msg.pose.header.frame_id = '/map'
        goal_pose_msg.pose.pose.position.x = goal_x
        goal_pose_msg.pose.pose.position.y = goal_y
        goal_pose_msg.pose.pose.orientation.z = goal_theta
        self.goal_handle_future = self.action_client.send_goal_async(goal_pose_msg)

    def runNavigation(self):
        while rclpy.ok():
            rclpy.spin_once(self.nav2_node)
            if self.goal_handle_future.done():
                goal_handle = self.goal_handle_future.result()
                if goal_handle.accepted:
                    print("Goal accepted.")
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.nav2_node, result_future)
                    result = result_future.result().result
                    if result!=False:
                        print("Goal execution done!")
                    else:
                        print("Goal execution failed!")
                    break
                else:
                    print("Goal rejected.")
                    break
    
    def closeNavActionServer(self):
        self.nav2_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    nav_handler = NavActionServer()
    nav_handler.setGoalPose(0.9 , 0.0 , 0.1)
    nav_handler.runNavigation()
    nav_handler.closeNavActionServer()
