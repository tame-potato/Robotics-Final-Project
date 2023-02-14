#!/usr/bin/env python
# license removed for brevity
# EECE_5560 Team Final Project: Quidditch Robots
# Edmundo Peralta
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *
import random

class Chaser:
    def __init__(self):
        # ------------ Subscriptions -------------
        self.sub = rospy.Subscriber('/robot_0/odom', Odometry, self.saveOdomSelf)       # chaser Team A
        self.sub = rospy.Subscriber('/robot_3/odom', Odometry, self.saveOdomRobot3)     # beater Team B
        self.sub = rospy.Subscriber('/robot_7/odom', Odometry, self.saveOdomRobot7)     # keeper Team B
        self.sub = rospy.Subscriber('/robot_8/odom', Odometry, self.saveOdomRobot8)     # quaffle
        self.sub = rospy.Subscriber('/robot_11/odom', Odometry, self.saveOdomRobot11)   # goal Team B (team A scores here)
#        self.subGame = rospy.Subscriber("Game", Announcements, self.callback_Announcements) # game Announcer, Need to know what is it and msg type / info
        # ------------ end of Subscriptions --------

        # Publish move commands
        self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)

        # ------- initialize objects odometry and twist and game flags -----
        self.my_odom = Odometry()
        self.robot_1_odom = Odometry()      # opposite team B chaser, may not use it for now
        self.robot_3_odom = Odometry()      # beater Team B odometry
        self.robot_7_odom = Odometry()      # keeper Team B odometry
        self.robot_8_odom = Odometry()      # quaffle odometry
        self.robot_11_odom = Odometry()     # goal Team B odometry
        self.twist = Twist()    
        self.gotthequaffle_flag= False      # reset flag to False on new game
        self.speed_mod = random.random() - 0.5
        # --------- end of initialization ----------------------

        # ----------- Callbacks -------------------
    def saveOdomSelf(self, msg):        # robot_0 chaser Team A
        self.my_odom = msg
    def saveOdomRobot3(self, msg):      # robot_3 beater Team B
        self.robot_3_odom = msg
    def saveOdomRobot7(self, msg):      # robot_7 keeper Team B
        self.robot_7_odom = msg
    def saveOdomRobot8(self, msg):      # robot_8 quaffle
        self.robot_8_odom = msg
    def saveOdomRobot11(self, msg):     # robot_11 Goal Team B
        self.robot_11_odom = msg
        # ----------- end of Callbacks ------------

    def get_twist_to_waypoint(self, waypoint_odom):
        x = self.my_odom.pose.pose.orientation.x
        y = self.my_odom.pose.pose.orientation.y
        z = self.my_odom.pose.pose.orientation.z
        w = self.my_odom.pose.pose.orientation.w
        heading = euler_from_quaternion([x, y, z, w])[2]
        bearing = np.arctan2((waypoint_odom.pose.pose.position.y - self.my_odom.pose.pose.position.y), (waypoint_odom.pose.pose.position.x - self.my_odom.pose.pose.position.x))
#        print('Heading:', heading)
#        print('Bearing:', bearing)
        if heading < bearing:
            return 1
        else:
            return -1

    def got_the_quaffle(self):
        # check if i got the quaffle (in close range), what is close range?
        x1 = self.my_odom.pose.pose.position.x  # self  Robot_0 chaser Team A                                             
        y1 = self.my_odom.pose.pose.position.y  # self  Robot_0 chaser Team A                                               
        x2 = self.robot_8_odom.pose.pose.position.x  # quaffle Robot_8
        y2 = self.robot_8_odom.pose.pose.position.y  # quaffle Robot_8
        
        distance= self.check_distance(x1, y1, x2, y2)
        #print("distance to quaffle:", distance)

        if ((x1 + y1 <> 0) and (x2 + y2 <>0)):      # on first launch, the robots position shows [0, 0], if not real, skeep
            if (distance <= 2):   # is this close enough without colliding?
            # I got it Yeahh! let's move to the goal ... 
                self.gotthequaffle_flag = True       # set and keep flag True for simulation, quaffle needs to follow
                return self.gotthequaffle_flag       # reset to false on new game
            else:
                self.gotthequaffle_flag = False       # set and keep flag True for simulation, quaffle needs to follow
                return self.gotthequaffle_flag       # reset to false on new game
                                    
    def check_if_scored(self):
        # check distance to the goal.. did I just score?
        x1 = self.my_odom.pose.pose.position.x          # self  Robot_0 chaser Team A                                             
        y1 = self.my_odom.pose.pose.position.y          # self  Robot_0 chaser Team A                                               
        x2 = self.robot_11_odom.pose.pose.position.x    # Robot_11 Goal Team B
        y2 = self.robot_11_odom.pose.pose.position.y    # Robot_11 Goal Team B
        distance=self.check_distance(x1, y1, x2, y2)
        #print("distance to goal:", distance)                           # check distance between chaser Team A and Goal Team B
        if distance <= 2:
            #print("Gooooooooaaaaaaaalllllllll")
            return True
        else:
            # not yet !!!
            return False

    def check_distance(self, x1, y1, x2, y2):
        # calculate distance between pointA and pointB
        pointA = [x1, y1]          # define pointA position
        pointB = [x2, y2]          # define pointB position
        distance = math.sqrt((pointA[0]- pointB[0]) **2 + (pointA[1]- pointB[1]) **2)   
        return distance

    def goto_location(self, targetrobot):
        # move to specified robot
        self.twist.linear.x = 2
        self.pub.publish(self.twist)
        time.sleep(.1)

        for i in range(3):
            self.twist.linear.x = 2 + self.speed_mod
            self.twist.angular.z = self.get_twist_to_waypoint(targetrobot)     # move to target robot
            self.pub.publish(self.twist)
            time.sleep(0.1)

    def process(self):
        # ------ Main logic ------------
        # check Announcements: New play?  (Needs to be defined)
        # do i have the Quaffle? go check
        self.got_the_quaffle()

        if self.gotthequaffle_flag:         # quaffle needs to follow
            #print("--- got the Quaffle!!!, now move let's score ---")
            # we got it, now let's make some point$$$$!!!, go to the goal and score ... victory!
            # avoid Beater & Keeper, maybe implement later, lets move the goal for now.
            self.goto_location(self.robot_11_odom)     # move to goal Team B Robot_12
            self.check_if_scored()                     # check if we scored.. (got to the goal)

        else:                 
            #print("I dont have the quaffle, let's go get it, goto quaffle")                         
            # I don't have the Quaffle yet, go get it ... move, move ...
            # avoid Beater & Keeper, maybe implement later, lets get the quaffle for now.
            self.goto_location(self.robot_8_odom)      # move to quaffle Robot_8
            
        # When does the play ends? what where to move to?

if __name__ == '__main__':
    rospy.init_node('chaser', anonymous=True)
    chaser = Chaser()
    while True:
        chaser.process()
