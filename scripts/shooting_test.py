#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizontal controls angular speed


def callback(inputData):

    if inputData.buttons[5] == 1:
        pub.publish(Empty())


# Initializes everything
def start():
    rospy.init_node('joystick_control')
    global pub
    pub = rospy.Publisher('turret_shoot', Empty, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()


if __name__ == '__main__':
    start()
