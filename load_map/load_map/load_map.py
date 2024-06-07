from nav2_msgs.srv import LoadMap
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class CallLoadMap(Node):

    def __init__(self):
        super().__init__('call_load_map_node')
        self.cli = self.create_client(LoadMap, 'map_server/load_map')
        self.init_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()
        self.declare_parameter('map_url_param', '')

    def send_request(self):
        self.req.map_url = self.get_parameter('map_url_param').get_parameter_value().string_value
        self.req.map_url = '/home/tony/map/room_2/map.yaml'
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def pubInitialPose(self, x, y , z = 0.0, theta = 1.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = '/map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = theta
        print("Initial pose set x: {}, y: {}, w: {}".format(x, y, theta))
        self.init_pose_publisher.publish(msg)
    
    def switchMap(self):
        response = self.send_request()
        if response:
            self.pubInitialPose(0.0, 0.0, 1.0)
        else:
            print("Initial fail")
    

def main():
    rclpy.init()

    load_map_client = CallLoadMap()
    load_map_client.switchMap()
    # response = load_map_client.send_request()
    # load_map_client.get_logger().info('response: ', response)

    load_map_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()