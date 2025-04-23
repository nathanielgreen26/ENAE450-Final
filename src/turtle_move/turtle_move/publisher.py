import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 3  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.flag=1
        print("init")
        #self.i = 0

    def timer_callback(self):

        msg = Twist()
        if self.flag==1 :
            msg.linear.x = 0.1
            print("in timercallback")

            self.publisher_.publish(msg)
            self.flag=0
        else:

            msg.linear.x = 0.0
            print("in timercallback")

            self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()