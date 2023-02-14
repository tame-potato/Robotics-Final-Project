#!/usr/bin/env python
#license removed for brevity
#Quidditch Robots - Gryffindor team aka Harry Potter
#Author: Meg Dillard

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from shared_utils.shared_functions import got_the_ball, goto_location, check_distance
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *
import random


class Seeker:
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

        x1 = self.my_odom.pose.pose.position.x        
        y1 = self.my_odom.pose.pose.position.y
        x2 = self.robot_10_odom.pose.pose.position.x
        y2 = self.robot_10_odom.pose.pose.position.y

        distance = check_distance(x1, y1, x2, y2)
        #print("distance to snitch:", distance)
        if distance <= 2:
            #print("GAME OVER - GRYFFINDOR WINS!")
            return True 
        else:
            return False

    def process(self):

        if self.gotthesnitch_flag is False:

            goto_location(self.robot_10_odom, self.my_odom, self.twist, self.pub)
            self.gotthesnitch_flag = got_the_ball(self.my_odom, self.robot_10_odom)

        elif self.gotthesnitch_flag is True:
            #print("GAME OVER - GRYFFINDOR WINS!")
            pass

if __name__ == '__main__':
    rospy.init_node('seeker1_odom', anonymous=True)
    seeker = Seeker()
    while True:
        seeker.process()
