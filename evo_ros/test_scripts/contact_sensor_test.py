#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ContactsState


def contact_test(contacts):
	
	# Check for contact states
	#	If none, pass
	#	If any contact, mark a collision
	if not contacts.states:
		pass
	else:
		print('Collision!')


rospy.init_node('Contact_Sensor_Test', anonymous=False)

scan_sub = rospy.Subscriber("/chassis_contact_sensor_state", ContactsState,contact_test)

rospy.spin()
