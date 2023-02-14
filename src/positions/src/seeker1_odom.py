#!/usr/bin/env python
#license removed for brevity
#Quidditch Robots - Gryffindor team aka Harry Potter
#Author: Meg Dillard
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *
import random


class Seeker1:
    def __init__(self):
        self.sub = rospy.Subscriber('/robot_4/odom', Odometry, self.saveOdomSelf)	#seeker for Gryffindor aka HP
        self.sub = rospy.Subscriber('/robot_10/odom', Odometry, self.saveOdomRobot10)		#Snitch 
        self.sub = rospy.Subscriber('/robot_5/odom', Odometry, self.saveOdomRobot5)		#Hufflepuff seeker (rival robot)

        self.pub = rospy.Publisher('/robot_4/cmd_vel', Twist, queue_size=10)	#publishes the move command for HP seeker bot

	#Initialization of bots and ball and Twist and flag
        self.my_odom = Odometry()
        self.robot_10_odom = Odometry()	#seeker odom
        self.robot_5_odom = Odometry()  #opposite team seeker
        self.twist = Twist()
        self.gotthesnitch_flag = False	#reset at new game

        #Callbacks from Subscribe Functions
    def saveOdomSelf(self, msg):
        self.my_odom = msg	#HP robot

    def saveOdomRobot10(self, msg):
        self.robot_10_odom = msg   #Snitch

    def saveOdomRobot5(self, msg):
        self.robot_5_odom = msg    #Opposing team seeker

#Begin Rotations and Distance Calculations

    def get_twist_to_waypoint(self, waypoint_odom):
        x = self.my_odom.pose.pose.orientation.x
        y = self.my_odom.pose.pose.orientation.y
        z = self.my_odom.pose.pose.orientation.z
        w = self.my_odom.pose.pose.orientation.w
        heading = euler_from_quaternion([x, y, z, w])[2]
        bearing = np.arctan2((waypoint_odom.pose.pose.position.y - self.my_odom.pose.pose.position.y), (waypoint_odom.pose.pose.position.x - self.my_odom.pose.pose.position.x))

        if heading < bearing:
            return 1
        else:
            return -1

    def got_the_snitch(self):
        x1 = self.my_odom.pose.pose.position.x
        y1 = self.my_odom.pose.pose.position.y
        x2 = self.robot_10_odom.pose.pose.position.x
        y2 = self.robot_10_odom.pose.pose.position.y

        distance = self.check_distance(x1, y1, x2, y2)
        #print("distance to snitch:", distance)

        if ((x1 + y1 <> 0 ) and (x2 + y2 <> 0)):
            if (distance <= 2):
                self.gotthesnitch_flag = True
                return self.gotthesnitch_flag


    def check_if_got_snitch(self):
        x1 = self.my_odom.pose.pose.position.x
        y1 = self.my_odom.pose.pose.position.y
        x2 = self.robot_10_odom.pose.pose.position.x
        y2 = self.robot_10_odom.pose.pose.position.y

        distance = self.check_distance(x1, y1, x2, y2)
        #print("distance to snitch:", distance)
        if distance <= 2:
            print("GAME OVER - GRYFFINDOR WINS!")
            return True 
        else:
            return False

    def check_distance(self, x1, y1, x2, y2):
        seeker1 = [x1, y1]
        snitch = [x2, y2]
        distance = math.sqrt((seeker1[0] - snitch[0]) **2 + (seeker1[1] + snitch[1]) **2)
        return distance

    def go_to_snitch(self, target):
        self.twist.linear.x = 2
        self.pub.publish(self.twist)
        time.sleep(0.1)

        for i in range(3):
            self.twist.linear.x = 2 + (random.random() - 0.5)
            self.twist.angular.z = self.get_twist_to_waypoint(target)
            self.pub.publish(self.twist)
            time.sleep(0.1)

    def process(self):
        self.got_the_snitch()

        if self.gotthesnitch_flag is False:
            self.go_to_snitch(self.robot_10_odom)
            self.check_if_got_snitch()
        elif self.gotthesnitch_flag is True:
            print("GAME OVER - GRYFFINDOR WINS!")

if __name__ == '__main__':
    rospy.init_node('seeker1_odom', anonymous=True)
    seeker1 = Seeker1()
    while True:
        seeker1.process()
