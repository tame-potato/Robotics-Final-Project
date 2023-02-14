#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from shared_utils.shared_functions import check_distance, get_twist_to_waypoint
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *

class Keeper:
    def __init__(self):
        self.sub = rospy.Subscriber('/robot_7/odom', Odometry, self.saveOdomSelf)         # subscribe to own (Even Keeper's) odometry
        self.sub = rospy.Subscriber('/robot_0/odom', Odometry, self.saveOdomRobot0)       # subscribe to Odd Chaser's odometry
        self.sub = rospy.Subscriber('/robot_11/odom', Odometry, self.saveOdomRobot11)     # subscribe to Even Goal
        self.pub = rospy.Publisher('/robot_7/cmd_vel', Twist, queue_size=10)              # publish cmd_vel to make Even Keeper move

        self.robot_7_odom = Odometry()                                                    # initialize messages for odometries and twist motion
        self.robot_0_odom = Odometry()
        self.robot_11_odom = Odometry()
        self.twist = Twist()

        self.BlockedChaser = False

        self.twist.linear.x = 1                                                            # initial movement for Stage
        self.pub.publish(self.twist)
        time.sleep(1)
        rospy.logwarn("initial movement done")        


    def saveOdomSelf(self, msg):                                                          # save own (Even Keeper's) odometry
        self.robot_7_odom = msg

    def saveOdomRobot0(self, msg):                                                        # save Odd Chaser's odometry
        self.robot_0_odom = msg

    def saveOdomRobot11(self, msg):                                                       # save Even Goal's odometry
        self.robot_11_odom = msg

    def updatePosition(self):
        self.x1 = self.robot_0_odom.pose.pose.position.x                                               
        self.y1 = self.robot_0_odom.pose.pose.position.y                                               
        self.x12 = self.robot_11_odom.pose.pose.position.x
        self.y12 = self.robot_11_odom.pose.pose.position.y
        self.x6 = self.robot_7_odom.pose.pose.position.x
        self.y6 = self.robot_7_odom.pose.pose.position.y
        self.chaser_position = [self.x1, self.y1]
        self.goal_position = [self.x12, self.y12]
        self.keeper_position = [self.x6, self.y6] 
        
    def process(self):
        #rospy.logwarn("start")
        self.twist.linear.x = 0                                                            # initial movement for Stage
        self.pub.publish(self.twist)
        time.sleep(1)
        #rospy.logwarn("initial movement done")

        while self.BlockedChaser is False:
            #rospy.logwarn("Entered first while loop")
            self.updatePosition()
            dist_bw_chaser_goal = check_distance(self.chaser_position[0], self.chaser_position[1], self.goal_position[0], self.goal_position[1])
            #print("The distance between the Chaser and the Goal is %f", dist_bw_chaser_goal)
            #rospy.logwarn(dist_bw_chaser_goal)
            if (dist_bw_chaser_goal > 10):
                continue
            elif (dist_bw_chaser_goal <= 10):
                self.updatePosition()
                dist_bw_keeper_chaser = check_distance(self.keeper_position[0], self.keeper_position[1], self.chaser_position[0], self.chaser_position[1])
                while (dist_bw_keeper_chaser > 0):                                       # while Even Keeper hasn't reached Odd Chaser
                    self.twist.linear.x = 2                                              # move towards Odd Chaser
                    self.twist.angular.z = get_twist_to_waypoint(self.robot_7_odom, self.robot_0_odom)
                    self.pub.publish(self.twist) 
                    time.sleep(0.2) 
                    self.updatePosition()
                    dist_bw_keeper_chaser = check_distance(self.keeper_position[0], self.keeper_position[1], self.chaser_position[0], self.chaser_position[1])
                    #print("The distance between the Keeper and Chaser is %f", dist_bw_keeper_chaser)
                    if (dist_bw_keeper_chaser == 0):                                     # when Even Keeper reaches Odd Chaser
                        #print("The Keeper has blocked the Chaser!")
                        break
                self.twist.linear.x = 0                                                  # Even Keeper stops
                self.pub.publish(self.twist)
                self.BlockedChaser = True
                break
             


# here gamekeeper needs to reset to original position?


if __name__ == '__main__':
    rospy.init_node('keeper_node', anonymous=True)
    keeper = Keeper()
    keeper.process()
    while not rospy.is_shutdown():
        rospy.spin()
    
