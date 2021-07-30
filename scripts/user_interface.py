#! /usr/bin/env python 

"""
.. module:: user_interface
:platform: Unix
:synopsis: Python module for the user Interface

.. moduleauthor:: Litong Huang

This node implements an user interface

Service:
/user_interface

"""

import rospy
import time
from rt2_ass2.srv import Command


def main():
	
	"""
	This function initializes the ROS node and waits for the user to
	insert *start* or *stop* to control the robot, by relying on the
	`rospy <http://wiki.ros.org/rospy/>`_ module.
	
	The user message is passed to the service ``user_interface``,
	advertised by :mod:`go_to_point`.
	
	"""
    rospy.init_node('user_interface')
    #initialization of the /user_interface client
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
			# Press 1, server is called to start the robot
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("The robot will stop")
            # Press 0, the server is called to stop the robot
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
