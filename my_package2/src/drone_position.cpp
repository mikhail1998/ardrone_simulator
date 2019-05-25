#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "iostream"
#include <math.h> 
using namespace std;
float x=0.00,y=0.00,z=0.04,dt,current_time,fixed_time,last_time;
int flag=1;
void recieve(const ardrone_autonomy::Navdata& a){
	if (flag==1){
		fixed_time=a.tm;
	}
	current_time=a.tm-fixed_time;
	dt=current_time-last_time;
	last_time=current_time;
	x+=a.vx*dt/1000.00*pow(10.00,-6.00);
	y+=a.vy*dt/1000.00*pow(10.00,-6.00);
	z+=a.vz*dt/1000.00*pow(10.00,-6.00);
	//cout << "x: " << x << "    " << "y: " << y << "    " << "z: " << z << "\n";
	flag=0;
	return;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/ardrone/navdata", 1000, recieve);
	ros::spin();
	return 0;
}
