#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
# To get the position of the robot and control it
# In Quaternion
from nav_msgs.msg import Odometry
# Transformation library to
# In raw, pitch, and yawn 
from tf import transformations
import math

# Robot state variables
position_ = Point()
yaw_ = 0

# Machine state
state_ = 0

# Goal
desired_position_ = Point()
desired_position_.x = -3
desired_position_.y = 7
desired_position_.z = 0

# Parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# Publisher to the cmd_vel
pub = None



# To get the position from odometry
def callback_odom(msg):
    global position_
    global yaw_

    # Position
    position_ = msg.pose.pose.position

    # Yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

    euler = transformations.euler_from_quaternion(quaternion)
    #roll = euler[0]
    #pitch = euler[1]
    yaw_ = euler[2]



def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_



def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_

    twist_msg = Twist()

    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)

    # State change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)



def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        #twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)
    
    # State change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)



# To stop the robot
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)



def main():
    global pub

    rospy.init_node('publish_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    #sub_laser = rospy.Subscriber('/m2wr/laser/scan', LaserScan, callback_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_== 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()



if __name__ == '__main__':
    main()