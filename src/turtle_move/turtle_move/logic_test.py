import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from typing import List
import time
from enum import Enum

class WallFollow(Node): 
    
    def __init__(self):
        super().__init__('wall_follow')
   	 
        self.subscription = self.create_subscription(
        	LaserScan,
        	'scan',
        	self.listener_callback,
        	10)
   	 
    	# Enums to help with keeping track of directions
        self.stages = Enum('Stages', 'TURNING_LEFT TURNING_RIGHT MOVING_FORWARD')
        self.stage = self.stages.MOVING_FORWARD
        self.walls = Enum('Walls', 'LEFT RIGHT')
        self.start_wall = None
   	 
    	# Fine tuning variables
        self.delay = 0.1
        self.wall_dist = 1.0
        self.forward_speed = 0.2
        self.turning_speed = 0.5
        self.side_threshold = 25
        self.front_threshold = 25
        self.corner_threshold = 5
   	 
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
   	 
        print("init")


    def listener_callback(self, msg):
   	 
        print("callback")
   	 
        values = msg.ranges
   	 
    	# Index values based on which direction we want to read in:
        left = values[480:600]
        front_left = values[440:460]
        front = values[330:390]
        front_right = values[260:280]
        right = values[120:240]

    	# If the movement has started:
        if self.start_wall:
        	# Check if front is hitting a wall
            front_obstacle = self.check_obstacle(front, self.front_threshold)
            left_obstacle = self.check_obstacle(left, self.side_threshold)
            front_left_obstacle = self.check_obstacle(front_left, self.corner_threshold, 0.2)
            front_right_obstacle = self.check_obstacle(front_right, self.corner_threshold, 0.2)
            right_obstacle = self.check_obstacle(right, self.side_threshold)
       	
            # If I am moving forward:
            if self.stage == self.stages.MOVING_FORWARD:
           	 
                print("Moving Forward")
           	 
            	# We want to check if the start_wall still exists to hug the wall while moving
                start_wall_obstacle = right_obstacle if self.start_wall == self.walls.RIGHT else left_obstacle
           	 
            	# If I don't see an obstacle in front:
                if not front_obstacle:
               	 
                    print("No wall ahead!")
               	 
                	# Adjust if I am going to hit a wall with any corners
                    # Turn right to avoid hitting left wall
                    if front_left_obstacle:
                        print("Adjust right")
                        self.turn_right()
                        self.stop()
                        time.sleep(self.delay*100)
                   	 
                	# Turn left to avoid hitting right wall                   	 
                    elif front_right_obstacle:
                        print("Adjust left")
                        self.turn_left()
                        self.stop()
                        time.sleep(self.delay*100)
               	 
                	# Continue moving forward if start_wall still is there
                    if start_wall_obstacle:
                        self.move_forward()
                	# Or else turn towards the start_wall
                    else:
                        print("Lost my wall!")
                        #self.turn_right() if self.start_wall == self.walls.RIGHT else self.turn_left()
                        self.move_forward_right() if self.start_wall == self.walls.RIGHT else self.move_forward_left() 
                        
           	 
            	# STOP! There is an obstacle in front!
                else:
                    print("Stop and turn!")
                    self.turn_left() if self.start_wall == self.walls.RIGHT else self.turn_right()
                      	 
        	# If I am already turning:    
            else:
                print("Turning")
           	 
            	# Based on the start_wall and current turning direction, we do different cases
                match (self.start_wall, self.stage):
               	 
                	# Hug left wall, turn left -> turn until I see left wall
                    case (self.walls.LEFT, self.stages.TURNING_LEFT):
                        print("Need to hug wall")
                        if left_obstacle or front_left_obstacle:
                            self.move_forward()
                	# Hug left wall, turn right -> turn until I see nothing in front
                    case (self.walls.LEFT, self.stages.TURNING_RIGHT):
                        if not front_obstacle:
                            self.move_forward()
                	# Hug right wall, turn right -> turn until I see right wall
                    case (self.walls.RIGHT, self.stages.TURNING_RIGHT):
                        print("Need to hug wall")
                        if right_obstacle or front_right_obstacle:
                            self.move_forward()
                	# Hug right wall, turn left -> turn until I see nothing in front
                    case (self.walls.RIGHT, self.stages.TURNING_LEFT):
                        if not front_obstacle:
                            self.move_forward()
           	 
   	 
    	# If I am starting mission:
        else:
            left_count = self.check_count(left)
            right_count = self.check_count(right)
            print(f"left: {left_count}, right: {right_count}")
       	 
            # Turn to whichever wall is closer and remember to follow that wall
            if right_count < left_count:
                print("Started turning left")
                self.turn_left()
                self.start_wall = self.walls.LEFT
            elif right_count > left_count:
                print("Started turning right")
                self.turn_right()
                self.start_wall = self.walls.RIGHT
            else:
                self.move_forward()
       	 
        time.sleep(self.delay)
    
    def move_forward(self):
        msg = Twist()
        self.stage = self.stages.MOVING_FORWARD
        msg.linear.x = self.forward_speed
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def move_forward_right(self):
        msg = Twist()
        self.stage = self.stages.MOVING_FORWARD
        msg.linear.x = self.forward_speed
        msg.angular.z = -self.turning_speed
        self.publisher_.publish(msg)

    def move_forward_left(self):
        msg = Twist()
        self.stage = self.stages.MOVING_FORWARD
        msg.linear.x = self.forward_speed
        msg.angular.z = self.turning_speed
        self.publisher_.publish(msg)
   	 
    def turn_right(self):
        msg = Twist()
        self.stage = self.stages.TURNING_RIGHT
        msg.linear.x = 0.0
        msg.angular.z = -self.turning_speed
        self.publisher_.publish(msg)
   	 
    def turn_left(self):
        msg = Twist()
        self.stage = self.stages.TURNING_LEFT
        msg.linear.x = 0.0
        msg.angular.z = self.turning_speed
        self.publisher_.publish(msg)
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    
	# Check if the current robot sees an obstacle at dir
	# Increasing limit decreases sensitivity to obstacle
    def check_obstacle(self, dir: List[float], limit: float, dist = 1.0):
        count = 0
        for val in dir:
            if 0.0 <= val < dist:
                count += 1
                if count > limit:
                    return 1
        return 0
    
	# Check to see "how much wall there is" in the dir range
    def check_count(self, dir: List[float], dist = 1.0):
        count = 0
        for val in dir:
            if 0.0 <= val < dist:
                count += 1

        return count

def main(args=None):
	rclpy.init(args=args)

	wall_follow = WallFollow()

	rclpy.spin(wall_follow)

	wall_follow.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()