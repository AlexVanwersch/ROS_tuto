#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

point = (6,6)

laser = list()
odom = Odometry()


def callback(data):

    global laser 
    
    laser = data.ranges

def callback_odom(data):

    global odom

    odom = data


    


if __name__ == '__main__':

    rospy.init_node('stop_motion', anonymous=True)

    sub = rospy.Subscriber("/scan", LaserScan, callback)
    sub2 = rospy.Subscriber("/odom", Odometry, callback_odom)
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


    while not rospy.is_shutdown():

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y 

        orientation_list = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion (orientation_list)

        a = (point[0]-x, point[1]-y)
        angle = np.arctan2(a[1],a[0])

        diff_angle = yaw - angle

        while abs(diff_angle) > 0.087:
            
            init_motor = Twist()
            init_motor.angular.z = 0.01

            pub.publish(init_motor)


    
        
