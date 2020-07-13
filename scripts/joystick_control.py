#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from airsoft_robot.msg import YawPitchMotors
import numpy as np

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joystick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizontal controls angular speed


def callback(inputData):
    output_data = YawPitchMotors()
    output_data.pitch_motor = inputData.axes[4] * inputData.axes[4] * inputData.axes[4] * 500
    output_data.yaw_motor = inputData.axes[0] * inputData.axes[0] * inputData.axes[0] * -500
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
