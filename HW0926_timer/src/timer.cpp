/*
 * hello.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: Roi Yehoshua
 */

#include "ros/ros.h"
#include <ctime>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "timer");

	ros::NodeHandle nh;
	ros::Rate loop_rate(1);

	//int count = 0;
	while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
	{
		time_t now = time(0);

		tm *ltm = localtime(&now);

		ROS_INFO_STREAM(ltm->tm_year+1900<<'/'<<1+ltm->tm_mon<<'/'<<ltm->tm_mday<<"/, "<<ltm->tm_hour<<':'<<ltm->tm_min<<':'<<ltm->tm_sec);

		ros::spinOnce(); // Allow ROS to process incoming messages
		loop_rate.sleep(); // Sleep for the rest of the cycle

		//count++;
	}

	return 0;
}



