#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include <iostream>
#include <ctime>
#include "std_msgs/Empty.h"
using namespace std;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		fixed_time_=time(NULL);
		sub_ = n_.subscribe("/ardrone/navdata", 1000, &SubscribeAndPublish::callback, this);
	}
	void callback(const ardrone_autonomy::Navdata& input)
	{
		geometry_msgs::Twist twist;
		current_time_ = time(NULL)-fixed_time_;
		if (current_time_<=3.0){
			twist.linear.z = 2.0;
		}
		else if ((current_time_>3.0) && (current_time_<=6.0)){
			twist.linear.z = 0.0;
		}
		else if ((current_time_>6.0) && (current_time_<=9.0)){
			twist.linear.x = 2.0;
		}
		else if ((current_time_>9.0) && (current_time_<=12.0)){
			twist.linear.x = 0.0;
		}
		else if ((current_time_>12.0) && (current_time_<=15.0)){
			twist.linear.y = 2.0;
		}
		else if ((current_time_>15.0) && (current_time_<=18.0)){
			twist.linear.y = 0.0;
		}
		else if ((current_time_>18.0) && (current_time_<=21.0)){
			twist.linear.x = -2.0;
		}
		else if ((current_time_>24.0) && (current_time_<=27.0)){
			twist.linear.x = 0.0;
		}
		else if ((current_time_>27.0) && (current_time_<=30.0)){
			twist.linear.y = -2.0;
		}
		else if ((current_time_>30.0) && (current_time_<=33.0)){
			twist.linear.y = 0.0;
		}
		else if ((current_time_>33.0) && (current_time_<=36.0)){
			twist.linear.z = -2.0;
		}
		else if ((current_time_>36.0) && (current_time_<=39.0)){
			twist.linear.z = 0.0;
		}
		twist.angular.z=-input.rotZ;
		twist.angular.y=-input.rotY;
		twist.angular.x=-input.rotX;
		pub_.publish(twist);
	}
private:
	time_t current_time_;
	time_t fixed_time_;
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "client");
	SubscribeAndPublish SAPObject;
	ros::spin();
	return 0;
}
