#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include <unistd.h> // delay of code execution (function: sleep(seconds))
#include <math.h>
#include "iostream"
#include "libxl.h"
#include <signal.h>
#include <vector>
#define PI 3.14159265

using namespace std;
using namespace libxl;

vector<float> v1,v2,v3;
int flag=1,N=6;
float x=0.00,y=0.00,z=0.04,dt,current_time,fixed_time,last_time,vxf,vyf,vzf,alpha;
float kp1,eps1;
Book* book = xlCreateBook();
Sheet* sheet = book->addSheet("Sheet1");
Format* format = book->addFormat();

float sum(vector<float> v){
	float s=0.00;
	int n = v.size();
	for(int i=0;i<n;i++){
		s+=v[i];
	}
	return s;
}
void mySigintHandler(int sig)
{
	book->save("/home/mikhail/test.xls",true);
	book->release();
	ros::shutdown();
}
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
		geometry_msgs::Twist p;
		alpha=acos(yg/sqrt(pow(yg,2.00)+pow(xg,2.00)))*180.00/PI;
		if (flag==1){
			fixed_time=a.tm;
			epslast1=alpha-a.rotZ;
		}
		if (flag<=N){
			v1.push_back(a.vx);
			v2.push_back(a.vy);
			v3.push_back(a.vz);
			vxf=sum(v1)/flag;
			vyf=sum(v2)/flag;
			vzf=sum(v3)/flag;
		}
		else {
			v1.push_back(a.vx);
			v2.push_back(a.vy);
			v3.push_back(a.vz);
			v1.erase(v1.begin());
			v2.erase(v2.begin());
			v3.erase(v3.begin());
			vxf=sum(v1)/N;
			vyf=sum(v2)/N;
			vzf=sum(v3)/N;
		}
		current_time=a.tm-fixed_time;
		dt=current_time-last_time;
		last_time=current_time;
		x+=vxf*dt/1000.00*pow(10.00,-6.00);
		y+=vyf*dt/1000.00*pow(10.00,-6.00);
		z+=vzf*dt/1000.00*pow(10.00,-6.00);
		eps1=alpha-a.rotZ;
		p.angular.z=kp1*eps1+kd1*(eps1-epslast1)/dt;
		epslast1=eps1;
		sheet->writeNum(flag,0,current_time*pow(10.00,-6.00),format);
		sheet->writeNum(flag,1,x,format);
		sheet->writeNum(flag,2,y,format);
		sheet->writeNum(flag,3,z,format);
		flag+=1;
		//pub_.publish(p);
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "subscriber", ros::init_options::NoSigintHandler);
	format->setNumFormat(NUMFORMAT_NUMBER_D2);
	SubscribeAndPublish SAPObject;
	signal(SIGINT, mySigintHandler);
	ros::spin();
	return 0;
}
