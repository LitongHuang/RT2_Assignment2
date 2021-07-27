#! /usr/bin/env python -tt

import rospy
import time
from rt2_ass1.srv import Command

def main():
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
