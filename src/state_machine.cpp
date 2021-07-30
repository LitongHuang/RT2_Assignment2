/** 
* 
*  \file state_machine.cpp
*  \brief This file implements the finite state machine behaviour
*
*  \author Litong Huang
*  \version 0.1
*  \date 29/07/2021
*  \details
*   
*  Subscribes to: <BR>
*	 None
*
*  Publishes to: <BR>
*	 /reach
     /time
*
*  Services: <BR>
*    /user_interface
* 
*  Client: <BR>
	 /position_server
*
*  Action Client: <BR>
*    /go_to_point
*
*  Description: <BR>
*    This node is a server for the user interface, it receives what the user
* 	 writes, and it acts depending on it. If the client ask for the random 
* 	 position behaviour it calls the server '/random_position' and it waits 
*  	 for it to be finished. It also checks if the client requests the behaviour
* 	 to stop. In that case, it cancels the previous goal and waits for the
* 	 next command. In the mean time, it also sends information to the user_interface
* 	 node. It publishes a topic when a goal has been reached (True) or cancelled
* 	 (False) so that the user_interface can keep track of the reached and cancelled
* 	 goals. Also whenevera goal is reached, it publishes on the topic '/time' the
* 	 time in seconds between the request of the goal and the completion of it.
*/

#include "ros/ros.h"
#include "rt2_ass2/Command.h"
#include "rt2_ass2/RandomPosition.h"
#include <rt2_ass2/go_to_pointAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <chrono>

bool start = false;		/* needed to know if the user asked to stop or not */
bool not_moving = true;	/* used to know wheather robot is already moving towards a goal or not*/
std_msgs::Bool reached;	/* Message to state wheather the goal was completed or cancelled   */

/**
 * \brief: It receives the commands from the 'user_interface' node
 * \param req: the command received from the client
 * \param res: not set
 * 
 * \return: true
 * 
 * This function sets the global variable start to true if the command 
 * received is "start" or it sets it to false if the command is different.
 */
bool user_interface(rt2_ass2::Command::Request &req, rt2_ass2::Command::Response &res)
	{
		// if the request command is start start is set to true
		if (req.command == "start")
			{
				start = true;
			}
		else 
			{
				// if the command is not start we set start to false
				start = false;
			}
		return true;
}

/**
 * \brief: The main function
 * 
 * \return: 0
 * 
 * This function initializes the ros node, the server, the clients, the publishers.
 * Then, if ros is running, it checks the global variable 'start'. If start 
 * is true, it also checks if the robot is already moving towards a goal 
 * with the global variable 'not_moving'. If the robot is not already moving,
 * a new goal is set, if on the other hand the robot was already going, it 
   checks if the goal is reached. In this case, information is published
 * on the topics '/time' and '/reach'. If the global variable 'start' is false, 
 * check if the robot is already moving. If that is the case, I cancel 
 * the goal and publish information on the '/reach' topic. If the robot is 
 * still (not moving) and the 'start' variable is false, the program does nothing and nothing will happen.
 */
int main(int argc, char **argv)
{
	// Initialization of the node
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	ros::NodeHandle n2;
	ros::NodeHandle n3;
	//initialization of the server
	ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n1.serviceClient<rt2_ass2::RandomPosition>("/position_server");
	actionlib::SimpleActionClient<rt2_ass2::go_to_pointAction> ac("/go_to_point");
	ros::Publisher pub=n2.advertise<std_msgs::Bool>("/reach", 1000);
    ros::Publisher pub_time=n3.advertise<std_msgs::Float32>("/time", 1000);
   
	rt2_ass2::RandomPosition rp;
	rp.request.x_max = 5.0;
	rp.request.x_min = -5.0;
	rp.request.y_max = 5.0;
	rp.request.y_min = -5.0;
   auto start_time=std::chrono::steady_clock::now();
   auto end_time=std::chrono::steady_clock::now();
   float elapsed_time;
   std_msgs::Float32 time_total;
	while(ros::ok())
		{
			ros::spinOnce();
			// if the user requested the robot to move
			if (start)
				{
					// if the robot is not moving yet
					if (not_moving)
						{
							// I call for a new goal
							client_rp.call(rp);
							rt2_ass2::go_to_pointGoal goal;
							goal.target_pose.header.frame_id = "base_link";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose.position.x = rp.response.x;
							goal.target_pose.pose.position.y = rp.response.y;
							goal.target_pose.pose.orientation.z = rp.response.theta;
							std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
							ac.sendGoal(goal);
							// start the timer to see how long it takes to complete the goal
							start_time=std::chrono::steady_clock::now();
							not_moving = false;
						}
					// if the robot was already moving
					else 
						{
							// I check the status and if the state is SUCCEEDED I set notgoing to true
							if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
								{
									std::cout << "Goal completed" << std::endl;
									not_moving= true;
									reached.data=true;
									// publish on te topic /reach True
									pub.publish(reached);
									// stop the timer
									end_time=std::chrono::steady_clock::now();
									// Calculate how long it took
									std::chrono::duration<double> elapsed_seconds = end_time-start_time;
									std::cout <<"elapsed time:" <<elapsed_seconds.count() <<"s\n";
									elapsed_time=float(elapsed_seconds.count());
									time_total.data=elapsed_time;
									// publish on the topic /time the time to reach the goal
									pub_time.publish(time_total);
									
								}
						}
				}
			// If the user requested the robot to stop
			else 
				{
					// if the robot is already moving towards a goal
					if (!not_moving)
						{
							// I cancel all previous goal and set not_moving to true
							ac.cancelAllGoals();
							std::cout << "Goal cancelled" << std::endl;
							not_moving= true;
							reached.data=false;
							// publish on te topic /reach False
							pub.publish(reached);
						}
				}
  

		}
   return 0;
}
