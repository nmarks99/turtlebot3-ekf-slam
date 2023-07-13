import numpy as np
import modern_robotics as mr
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from nuturtle_control.srv import CsvTwist

LOG = False     # TODO: make this a parameter to the node
FREQ: int = 100 # TODO: make this a parameter to the node


class PlayCsv(Node):
    def __init__(self):
        super().__init__("play_csv")

        # Create timer
        _timer = self.create_timer(1/FREQ, self.timer_callback)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Services
        _play_csv_srv = self.create_service(CsvTwist, "csv_twist", self.csv_callback)
        
        self.CSV_RECIEVED = False
        self.twist_arr = np.array([])
        self.i = 0

    def csv_callback(self, request, response):
        self.CSV_RECIEVED = True
        csv_path = request.csv_path
        self.twist_arr = np.loadtxt(csv_path,delimiter=",")
        # self.get_logger().info(f"Array shape = {np.shape(self.twist_arr)}")
        return response


    def timer_callback(self):
        # self.get_logger().info(f"i = {self.i}")
        if self.CSV_RECIEVED:
            twist = self.twist_arr[self.i]
            twist_msg = Twist()
            twist_msg.angular.z = twist[0]
            twist_msg.linear.x = twist[1]
            twist_msg.linear.y = twist[2]
            self.cmd_vel_pub.publish(twist_msg)
            self.i += 1

        if self.i == np.shape(self.twist_arr)[0] - 1:
            # self.get_logger().info(f"Reached end of CSV")
            self.i = 0
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.CSV_RECIEVED = False

def main(args=None):
    '''
    main function/node entry point
    '''
    rclpy.init(args=args)
    node = PlayCsv()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()



