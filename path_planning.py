import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

import random
import math
import numpy as np
# The purpose of this node is to build off of dumb_vacuum to be slightly less dumb
# the proposed way of doing this, is to instead of looking straight forward, and viewing through a small
# field of view that leads to the robot "sticking" to the edges, what if we used the whole lidar, found the max 
# in a sliding window, say of 10 degrees, and then move in the center of that direction
# if there are multiple sliding windows that are optimal, we choose arbitrarily. 
# this adds a sense of randomness and might become completeness?!? total guess, but hopefully better

# 

class path_planning(Node):
    def __init__(self):
        super().__init__('test')

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
            
        self.odom_sub = self.create_subscription(
            Odometry, 
            'odom',
            self.odom_callback,
            10
        )

        self.map_sub = self.create_subscription(
            map, 
            'map',
            self.map_callback,
            10
        )
        self.compute_new_set = False
        self.angle_setpoint = 0.0

        # Store odometry so ET can find home. Effecively, init will be goal during pathing.
        # I think init and final should suffice as inputs to a path planning algorithm.
        self.initx = None
        self.inity = None
        self.finalx = None
        self.finaly = None

        # But when should ET go home? After the vacuum is done. We assume that when the program
        # runs, vacuuming is occuring up until a certain point.
        self.vacuuming = True

        # But when do we set self.vacuuming to false and use path planning?
        
        self.laser_sub  # prevent unused variable warning
        self.odom_message = Odometry()
    
    def odom_callback(self,msg): 
        self.odom_message = msg

        
        if self.initx == None or self.inity = None :
            self.initx = self.odom_message.pose.pose.position.x
            self.inity = self.odom_message.pose.pose.position.y
         
        self.finalx = self.odom_message.pose.pose.position.x
        self.finaly = self.odom_message.pose.pose.position.y
 

    


def main(args=None):
    rclpy.init(args=args)
    test = dumb_vacuum_v2()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
