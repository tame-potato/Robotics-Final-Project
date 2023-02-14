#!/usr/bin/env python
#license removed for brevity

#This robot is on the Gryffindor team which is even thus it listens for the seeker and the opposing team seeker which is robot5
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *

class Seeker:
    def __init__(self):
        self.sub = rospy.Subscriber('/robot_4/odom', Odometry, self.saveOdomSelf) #Subscribe to own position 
        self.sub = rospy.Subscriber('/robot_10/odom', Odometry, self.saveOdomRobot10) #Subscriber to snitch position 
        self.sub = rospy.Subscriber('/robot_5/odom', Odometry, self.saveOdomRobot5) #Subscriber to other seeker on opposing team

        self.pub = rospy.Publisher('/robot_4/cmd_vel', Twist, queue_size=10)

        self.my_odom = Odometry()
        self.robot_10_odom = Odometry()
        self.robot_5_odom = Odometry()

    def get_twist_to_waypoint(self, waypoint_odom):
        x = self.my_odom.pose.pose.orientation.x
        y = self.my_odom.pose.pose.orientation.y
        z = self.my_odom.pose.pose.orientation.z
        w = self.my_odom.pose.pose.orientation.w
        heading = euler_from_quaterion([x, y, z, w])[2]
        bearing = np.arctan2((waypoint_odom.pose.pose.position.y - self.my_odom.pose.pose.position.y), (waypoint_odom.pose.pose.position.x - self.my_odom.pose.pose.position.x))
        print('Heading: ', heading)
        print('Bearing: ', bearing)
        if heading < bearing:
            return 1
        else:
            return -1

    def process(self):
        twist = Twist()
        twist.linear.x = 1
        self.pub.publish(twist)
        time.sleep(1)

        x1 = self.robot_4_odom.pose.pose.position.x
        y1 = self.robot_4_odom.pose.pose.position.y
        x2 = self.robot_10_odom.pose.pose.position.x
        y2 = self.robot_10_odom.pose.pose.position.y
        x3 = self.robot_5_odom.pose.pose.position.x
        y3 = self.robot_5_odom.pose.pose.position.y

        seeker1_position = [x1, y1]
        snitch_position = [x2, y2]
        seeker2_position = [x3, y3]

        dist_to_snitch = math.dist(seeker1_position, snitch_position)

        while (dist_to_seekers > 10):
            self.twist.linear.x = 2
            self.twist.angular.z = self.get_twist_to_waypoint(self.robot_4_odom)
            self.pub.publish(twist)
            time.sleep(0.2)
            dist_to_snitch = math.dist(seeker1_position, snitch_position)
            if (dist_to_snitch <= 10):
                self.twist.linear.x = 3
                self.twist.angular.z = self.get_twist_to_waypoint(self.robot_4_odom)
                self.pub.publish(twist)
                time.sleep(0.2)
                dist_to_snitch = math.dist(seeker1_position, snitch_position)
            elif (dist_to_snitch == 0):
                break

         self.twist.linear.x = 0
         self.pub.publish(twist)

    def saveOdomSelf(self, msg):
        self.my_odom = msg

    def saveOdomRobot10(self, msg):
        self.robot_10_odom = msg

    def saveOdomRobot5(self, msg):
        self.robot_5_odom = msg


if __name__ == '__main__':
    try:
        rospy.init_node('seeker_odom', anonymous=True)
        seeker1 = Seeker()
        while True:
            seeker1.process()
    except rospy.ROSInterruptException:
        pass
