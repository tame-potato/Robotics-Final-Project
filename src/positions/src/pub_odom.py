#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import numpy as np
from tf.transformations import *

<<<<<<< HEAD
class Template:
    def __init__(self):
        self.sub = rospy.Subscriber('/robot_0/odom', Odometry, self.saveOdomSelf)
        self.sub = rospy.Subscriber('/robot_1/odom', Odometry, self.saveOdomRobot1)
        self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)

        self.my_odom = Odometry()
        self.robot_1_odom = Odometry()
=======
def talker():
    pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    twist = Twist()
    twist.linear.x = 1
    pub.publish(twist)
    pub2.publish(twist)
    time.sleep(1)
>>>>>>> 048af5d1d3a6addc175d6bf95e22f9ee8937eb88

    def get_twist_to_waypoint(self, waypoint_odom):
        x = self.my_odom.pose.pose.orientation.x
        y = self.my_odom.pose.pose.orientation.y
        z = self.my_odom.pose.pose.orientation.z
        w = self.my_odom.pose.pose.orientation.w
        heading = euler_from_quaternion([x, y, z, w])[2]
        bearing = np.arctan2((waypoint_odom.pose.pose.position.y - self.my_odom.pose.pose.position.y), (waypoint_odom.pose.pose.position.x - self.my_odom.pose.pose.position.x))
        print('Heading:', heading)
        print('Bearing:', bearing)
        if heading - bearing < -np.pi:
            return -1
        elif heading - bearing < 0:
            return 1
        elif heading - bearing < np.pi:
            return -1
        else:
            return 1

    def process(self):
        twist = Twist()
        twist.linear.x = 1
<<<<<<< HEAD
        self.pub.publish(twist)
        time.sleep(1)

        for i in range(10):
            twist = Twist()
            twist.linear.x = 2
            twist.angular.z = self.get_twist_to_waypoint(self.robot_1_odom)
            self.pub.publish(twist)
            time.sleep(0.2)

        twist = Twist()
        twist.linear.x = 0
        self.pub.publish(twist)

    def saveOdomSelf(self, msg):
        self.my_odom = msg

    def saveOdomRobot1(self, msg):
        self.robot_1_odom = msg
=======
        pub.publish(twist)
        pub2.publish(twist)
        time.sleep(1)

    twist = Twist()
    twist.linear.x = 0
    pub.publish(twist)
    pub2.publish(twist)
>>>>>>> 048af5d1d3a6addc175d6bf95e22f9ee8937eb88
 
if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        template = Template()
        while True:
            template.process()
    except rospy.ROSInterruptException:
        pass
