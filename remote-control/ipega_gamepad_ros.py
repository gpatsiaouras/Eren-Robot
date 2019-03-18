#!/usr/bin/env python

"""
Ipega gamepad cmd_vel published
"""

__author__ =  'Giorgos Patsiaouras <giorgospatsiaouras@gmail.com>'
__license__ = 'GPLv3'

import rospy
import pygame
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


MAX_VELOCITY = 23.1 * 0.0325
MAX_ANGULAR_SPEED = MAX_VELOCITY / 0.075

pygame.init()
# Just the first joystick
j = pygame.joystick.Joystick(0)
# Initiate
j.init()
# Debug to user
print 'Initialized Joystick : %s' % j.get_name()

# Receives data from the gamepad
def publisher():
	# Looping at
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		pygame.event.pump()

		# Empty values arrays
		axes_values = []
		buttons_values = []

		# Initiate the two messages
		msg_twist = Twist()
		msg_joy = Joy()

		# Iterate axes and populate value arrays
		for axis in range(j.get_numaxes()):
			axes_values.insert(axis, j.get_axis(axis))
		for button in range(j.get_numbuttons()):
			buttons_values.insert(button, j.get_button(button))

		msg_twist.linear.x = axes_values[1]*(-1) * MAX_VELOCITY
		msg_twist.angular.z = axes_values[2]*(-1) * MAX_ANGULAR_SPEED

		msg_joy.axes = axes_values
		msg_joy.buttons = buttons_values

		cmd_vel_pub.publish(msg_twist)
		joy_pub.publish(msg_joy)

		# Preserve looping in specified rate
		rate.sleep()


if __name__ == '__main__':
	# Initiate Node
	rospy.init_node('teleop_rojo_control', anonymous=True)
	# Publishers
	cmd_vel_pub = rospy.Publisher('/eren/cmd_vel', Twist, queue_size=1)
	joy_pub = rospy.Publisher('/eren/joy_teleop', Joy, queue_size=1)
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
