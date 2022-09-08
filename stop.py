#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

laser = list()

def callback(data):

    global laser 
    
    laser = data.ranges
    




if __name__ == '__main__':

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('stop_motion', anonymous=True)

    sub = rospy.Subscriber("/scan", LaserScan, callback)
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    init_motor = Twist()
    init_motor.linear.x = 0.01

    rospy.sleep(2)


    while not rospy.is_shutdown():

        

        if min(laser) < 0.5 :
            init_motor.linear.x = 0
            print("STOP")
            rospy.on_shutdown()
        else:
            init_motor.linear.x = 0.05
            


        pub.publish(init_motor)


    
        
