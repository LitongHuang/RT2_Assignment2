#include "ros/ros.h"
#include "rt2_ass1/RandomPosition.h"

// funtion to obtain a random number between minimum and maximum values for x and y
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

// when the request from the client exixts, the function called 
bool myrandom (rt2_ass1::RandomPosition::Request &req, rt2_ass1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


int main(int argc, char **argv)
{
   // initialization of the node 'position server'
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   // initialization of the server
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
