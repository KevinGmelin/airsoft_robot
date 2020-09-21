#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty


def callback(inputData):

    if inputData.buttons[5] == 1:
        pub.publish(Empty())


# Initializes everything
def start():
    rospy.init_node('shooting_test')
    global pub
    pub = rospy.Publisher('turret_shoot', Empty, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()


if __name__ == '__main__':
    start()