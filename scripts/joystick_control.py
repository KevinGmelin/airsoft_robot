#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from airsoft_robot.msg import YawPitchMotors
import numpy as np


def callback(inputData):
    output_data = YawPitchMotors()
    output_data.pitch_motor = int(inputData.axes[4] * inputData.axes[4] * inputData.axes[4] * 500)
    output_data.yaw_motor = int(inputData.axes[0] * inputData.axes[0] * inputData.axes[0] * -500)
    pub.publish(output_data)


# Initializes everything
def start():
    rospy.init_node('joystick_control')
    global pub
    pub = rospy.Publisher('arduinoIn', YawPitchMotors, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()


if __name__ == '__main__':
    start()