#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None
regions_ = {
        'right': 0,
        'front_right': 0,
        'front': 0,
        'front_left': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'front_right': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'front_left':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1.5
    
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 3 - front_right'
        change_state(2)
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 4 - front_left'
        change_state(0)
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 5 - front and front_right'
        change_state(1)
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 6 - front and front_left'
        change_state(1)
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 7 - front and front_left and front_right'
        change_state(1)
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 8 - front_left and front_right'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    # Turning to the right
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    # Turning to the left
    msg.angular.z = 0.3
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        msg = Twist()

        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()