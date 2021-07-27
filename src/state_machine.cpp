#include "ros/ros.h"
#include "rt2_ass1/Command.h"
#include "rt2_ass1/RandomPosition.h"
#include <rt2_ass1/go_to_pointAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// variable start: the user asked to start or not
bool start = false;

// variable not_moving: robot reaching the goal or not
bool not_moving = true;

// function that calls the server
bool user_interface(rt2_ass1::Command::Request &req, rt2_ass1::Command::Response &res)
	{
		// True if the request command is 'start' at first
		if (req.command == "start")
			{
				start = true;
			}
		else 
			{
				// if the command doesn't start with 'start'
				start = false;
			}
		return true;
}


int main(int argc, char **argv)
{
	// Initialization of the node
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	
	//initialization of the server
	ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n1.serviceClient<rt2_ass1::RandomPosition>("/position_server");
	actionlib::SimpleActionClient<rt2_ass1::go_to_pointAction> ac("/go_to_point");
   
   	// min and max for the x and y values
	rt2_ass1::RandomPosition rp; 
	rp.request.x_max = 5.0;
	rp.request.x_min = -5.0;
	rp.request.y_max = 5.0;
	rp.request.y_min = -5.0;
   
	while(ros::ok())
		{
			ros::spinOnce();
			// robot is instructed to move
			if (start)
				{
					// if the robot not moving yet
					if (not_moving)
						{
							// call for a new goal
							client_rp.call(rp);
							rt2_ass1::go_to_pointGoal goal;
							goal.target_pose.header.frame_id = "base_link";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose.position.x = rp.response.x;
							goal.target_pose.pose.position.y = rp.response.y;
							goal.target_pose.pose.orientation.z = rp.response.theta;
							std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
							ac.sendGoal(goal);
							not_moving = false;
						}
					// if the robot moves
					else 
						{
							// check if the status of the state is SUCCEEDED, robt stops after reaching the goal
							if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
								{
									std::cout << "Goal completed" << std::endl;
									not_moving= true;
								}
						}
				}
			// robot is required to stop
			else 
				{
					// if the robot is already moving towards a goal
					if (!not_moving)
						{
							// cancel the goal, and robot not moving 
							ac.cancelAllGoals();
							std::cout << "Goal cancelled" << std::endl;
							not_moving= true;
						}
				}
  

		}
   return 0;
}
