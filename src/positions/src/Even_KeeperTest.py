#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *

class EvenKeeper:
    def __init__(self):
        self.sub = rospy.Subscriber('/robot_2/odom', Odometry, self.saveOdomSelf)         # subscribe to own (Even Keeper's) odometry
        self.sub = rospy.Subscriber('/robot_1/odom', Odometry, self.saveOdomRobot1)       # subscribe to Odd Chaser's odometry
        self.sub = rospy.Subscriber('/robot_0/odom', Odometry, self.saveOdomRobot0)     # subscribe to Even Goal
        self.pub = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)              # publish cmd_vel to make Even Keeper move
        self.pub_test = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)

        self.robot_2_odom = Odometry()                                                    # initialize messages for odometries and twist motion
        self.robot_1_odom = Odometry()
        self.robot_0_odom = Odometry()
        self.twist = Twist()
        self.twist_test = Twist()

    def saveOdomSelf(self, msg):                                                          # save own (Even Keeper's) odometry
        self.robot_2_odom = msg

    def saveOdomRobot1(self, msg):                                                        # save Odd Chaser's odometry
        self.robot_1_odom = msg

    def saveOdomRobot0(self, msg):                                                       # save Even Goal's odometry
        self.robot_0_odom = msg

    def get_twist_to_waypoint(self, waypoint_odom):
        x = self.robot_2_odom.pose.pose.orientation.x
        y = self.robot_2_odom.pose.pose.orientation.y
        z = self.robot_2_odom.pose.pose.orientation.z
        w = self.robot_2_odom.pose.pose.orientation.w
        heading = euler_from_quaternion([x, y, z, w])[2]
        bearing = np.arctan2((waypoint_odom.pose.pose.position.y - self.robot_2_odom.pose.pose.position.y), (waypoint_odom.pose.pose.position.x - self.robot_2_odom.pose.pose.position.x))
        #print('Heading:', heading)
        #print('Bearing:', bearing)
        if heading - bearing < -np.pi:
            return -1
        elif heading - bearing < 0:
            return 1
        elif heading - bearing < np.pi:
            return -1
        else:
            return 1

    def calculateDistance(self, xA, yA, xB, yB):
        dist = math.sqrt((xB - xA) ** 2 + (yB - yA) ** 2)
        return dist


    def process(self):
        self.twist.linear.x = 1                                                            # initial movement for Stage
        self.pub.publish(self.twist)
        time.sleep(1)

        self.twist_test.linear.x = 1                                                            # initial movement for Stage
        self.pub_test.publish(self.twist_test)
        time.sleep(1)


        x1 = self.robot_1_odom.pose.pose.position.x                                               
        y1 = self.robot_1_odom.pose.pose.position.y                                               
        x0 = self.robot_0_odom.pose.pose.position.x
        y0 = self.robot_0_odom.pose.pose.position.y
        x2 = self.robot_2_odom.pose.pose.position.x
        y2 = self.robot_2_odom.pose.pose.position.y
        chaser_position = [x1, y1]                                                  # define point position of Odd Chaser
        goal_position = [x0, y0]                                                  # define point postion of Even Goal
        keeper_position = [x2, y2]                                                  # define point position of Even Keeper
        
        #dist_bw_chaser_goal = math.dist(chaser_position, goal_position)             # calculate distance between Odd Chaser and Even Goal
        dist_bw_chaser_goal = self.calculateDistance(x1, y1, x0, y0)

      #  while (dist_bw_chaser_goal > 0):
      #      twist_test.linear.x = 2
      #      twist_test.angular.z = self.get_twist_to_waypoint(self.robot_0_odom)
      #      self.pub_test.publish(twist_test)
      #      time.sleep(0.2)


        if (dist_bw_chaser_goal <= 10):
            #dist_bw_keeper_chaser = math.dist(keeper_position, chaser_position)
            dist_bw_keeper_chaser = self.calculateDistance(x2, y2, x1, y1)
            while (dist_bw_keeper_chaser > 0):                                       # while Even Keeper hasn't reached Odd Chaser
                self.twist.linear.x = 2                                              # move towards Odd Chaser
                self.twist.angular.z = self.get_twist_to_waypoint(self.robot_1_odom)
                self.pub.publish(self.twist) 
                time.sleep(0.2) 
                #dist_bw_keeper_chaser = math.dist(keeper_position, chaser_position) # calculate distance between Even Keeper, Odd Chaser again
                dist_bw_keeper_chaser = self.calculateDistance(x2, y2, x1, y1)
                if (dist_bw_keeper_chaser == 0):                                     # when Even Keeper reaches Odd Chaser
                    print("Keeper has stopped Chaser!")
                    break
            self.twist.linear.x = 0                                                  # Even Keeper stops
            self.pub.publish(self.twist)

    def moveChaser(self):
        while (dist_bw_chaser_goal > 0):
            self.twist_test.linear.x = 2
            self.pub_test.publish(self.twist_test)
            time.sleep(0.2)
        if (dist_bw_keeper_chaser == 0):
            self.twist_test.linear.x = 0                                                  # Even Keeper stops
            self.pub_test.publish(self.twist_test)   


# here gamekeeper needs to reset to original position?


if __name__ == '__main__':
    rospy.init_node('keeper_node', anonymous=True)     
    evenkeeper = EvenKeeper()
    evenkeeper.moveChaser()
    evenkeeper.process()
    rospy.spin()



#    try:
#        rospy.init_node('keeper_node', anonymous=True)
#        evenkeeper = EvenKeeper()
#        while True:
#            evenkeeper.process()
#    except rospy.ROSInterruptException:
#        pass
