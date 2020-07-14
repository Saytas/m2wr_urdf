#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback_laser(msg):
    ## 720 laser scanning from right to the left
    ## divided into 5 regions with 144 laser scannings each
    laser_reading_regions = [
        min(min(msg.ranges[0:143]), 10),
        min(min(msg.ranges[144:287]), 10),
        min(min(msg.ranges[288:431]), 10),
        min(min(msg.ranges[432:575]), 10),
        min(min(msg.ranges[576:719]), 10)
    ]
    ## Print the message
    rospy.loginfo(laser_reading_regions)
    ## rospy.loginfo(msg)

def main():
    ## Initializing the node and passing the topic name in it
    rospy.init_node("reading_laser")

    ## Subscribe to the Laser Scannings and passing the topic name
    ## and message type (data type), and define a callback in it
    sub = rospy.Subscriber("/m2wr/laser/scan", LaserScan, callback_laser)

    ## In order to keep the script running where we receive the messages
    ## we have to spin
    rospy.spin()

if __name__ == "__main__":
    main()