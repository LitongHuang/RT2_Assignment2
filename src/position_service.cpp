/** 
* 
*  \file position_service.cpp
*  \brief This file implements a position service
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
*	 None
*
*  Services: <BR>
*    /position_server
*
*  Action Services: <BR>
*    None
*
*  Description: 
*    This node 'position service' implements a position server. When the service is required, 
* the request containing the min and max values for the x and y position is used
* to generate a random position.
* 
* 
*/

#include "ros/ros.h"
#include "rt2_ass2/RandomPosition.h"


/**
 * \brief: It generates a random number
 * \param M: minimum possible number of the random number 
 * \param N: maximum possible number of the random number 
 * 
 * \return: the random number generated
 * 
 * This function uses the library function rand() to generate a random number and
 *  then resizes it to be in the interval [M;N].
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * \brief: Called when I have a request from the client (server callback)
 * \param req: it is a message composed of the maximum and minimum of the intervalin both x and y
 * \param res: it is a message composed of the random x and y coordinates and the orientation
 * 
 * \return: always True
 * 
 * This function calls the function randMtoN to generate three random numbers,
 * the x coordinate, the y coordinate and the orientation that is said to be
 * requested between -pi and pi.
 */
bool myrandom (rt2_ass2::RandomPosition::Request &req, rt2_ass2::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 * \brief: The main function
 * 
 * \return: 0
 * 
 * This function initializes the ros node and the server '/position_server',
 * it then runs waiting for a request for the server.
 */
int main(int argc, char **argv)
{
	// initialization of the node
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   // initialization of the server
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
