#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		sub_ = n_.subscribe("/ardrone/navdata", 1000, &SubscribeAndPublish::callback, this);
	}
	void callback(const ardrone_autonomy::Navdata& input)
	{
		geometry_msgs::Twist mrk;
		mrk.angular.z=-input.rotZ;
		mrk.angular.y=-input.rotY;
		mrk.angular.x=-input.rotX;
		pub_.publish(mrk);
	}
private:
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
