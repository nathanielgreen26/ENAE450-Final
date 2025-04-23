import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class MinimalSubscriber(Node):

 
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.stage = 1
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        print("init")
        

    def listener_callback(self, msg):
        print("callback")
        values = msg.ranges
        #print(values)
        max_val_ind = len(values) - 1
        front = values[330:390]
        right = values[150:210]
        left = values[510:570]
        back = values[-30:30]
        print(right)
        new_msg = Twist()
        new_msg.linear.x = 0.1
        new_msg.angular.z = 0.0
        count = 0
        if self.stage == 1:
            for val in front:
                if 0.0 <= val < 0.5:
                    count+=1
                    if count > 25:
                        new_msg.linear.x = 0.0
                        new_msg.angular.z = -0.5
                        self.publisher_.publish(new_msg)
                        time.sleep(1)
                        #self.stage = 2
                        break

                    
        
        """if self.stage ==2:
            new_msg.linear.x = 0.0
            new_msg.angular.z = -1.57/3.0
            self.publisher_.publish(new_msg)
            time.sleep(3)
            new_msg.angular.z = 0.0
            self.stage = 3

        if self.stage == 3:
            new_msg.linear.x = 0.0
            new_msg.angular.z = 0.0"""
        
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()