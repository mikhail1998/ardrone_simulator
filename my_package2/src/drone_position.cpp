#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include <unistd.h> // delay of code execution (function: sleep(seconds))
#include <math.h>
#include "iostream"
int flag=1;
float x=0.00,y=0.00,z=0.04,dt,current_time,fixed_time,last_time,vxf,vyf,vzf,alpha=0.25;
using namespace std;
class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		sub_ = n_.subscribe("/ardrone/navdata", 1000, &SubscribeAndPublish::callback, this);
	}
	void callback(const ardrone_autonomy::Navdata& a)
	{
		if (flag==1){
			fixed_time=a.tm;
			//vxf=a.vx;
			//vyf=a.vy;
			//vzf=a.vz;
		}
		else {
			vxf*=(1.00-alpha);
			vxf+=alpha*a.vx;
			vyf*=(1.00-alpha);
			vyf+=alpha*a.vy;
			vzf*=(1.00-alpha);
			vzf+=alpha*a.vz;
		}
		current_time=a.tm-fixed_time;
		dt=current_time-last_time;
		last_time=current_time;
		x+=vxf*dt/1000.00*pow(10.00,-6.00);
		y+=vyf*dt/1000.00*pow(10.00,-6.00);
		z+=vzf*dt/1000.00*pow(10.00,-6.00);
		//x+=a.vx*dt/1000.00*pow(10.00,-6.00);
		//y+=a.vy*dt/1000.00*pow(10.00,-6.00);
		//z+=a.vz*dt/1000.00*pow(10.00,-6.00);
		cout << "x: " << x << "    " << "y: " << y << "    " << "z: " << z << "\n";
		//cout << "vx= " << a.vx << "    vxf= " << vxf << "\n";
		flag=0;
		//pub_.publish(mrk);
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "subscriber");
	SubscribeAndPublish SAPObject;
	ros::spin();
	return 0;
}
