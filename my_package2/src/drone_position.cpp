#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include <unistd.h>
#include <math.h>
#include "iostream"
#include "libxl.h"
#include <signal.h>
#include <vector>

#define PI 3.14159265

using namespace std;
using namespace libxl;

vector<float> v1,v2,v3,v4,v5,v6;
int flag=1,counter=1,flag2=1;
double x=0.00,y=0.00,z,dt,current_time,fixed_time,last_time=0.00,vxf,vyf,vzf,alphaX;
double kp1=8.0,eps1,last_alpha,w=0.00,alphaf,wzmax=1.5,eps1_integral=0.00,ki1=0.00,goalw;
double deps1=0.0,kd1=0.00,epslast1,vxg,vyg,xgoal,ygoal,zgoal,epsx,eps1max=0.5,kp2=5.0;
const double vxmax=2.0,vymax=2.0,vzmax=0.5,kp3=1.5,kp4=0.45,kpx=5.0,ky=8.0,kpz=12.0,kd2=1.8;
double epsz,epsy,koef,vxgoal,vzgoal,vygoal,dist,epsxlast,depsx,epszlast,depsz,kdz=2.5;
const float N=6.00,N1=15.00;
double bettaf,gamaf,deps2,deps3,epslast2,epslast3,kdx=2.0,kdy=2.0,eps2,eps3;
double xglocal,yglocal,zglocal,koef1,kp22=8.0;
char response,input='y';
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
int sign(double value){
	if(value>0.0){
		return 1;
	}
	else if (value==0.0){
		return 0;
	}
	else {
		return -1;
	}
}
void mySigintHandler(int sig)
{
	cout << "Do you want exit[y/n]: ";
	cin >> response;
	if(response==input){
		book->save("/home/mikhail/test.xls",true);
		book->release();
		ros::shutdown();
	}
	else {
		cout << "Enter xgoal: ";
		cin >> xgoal;
		cout << "\n" << "Enter ygoal: ";
		cin >> ygoal;
		cout << "\n" << "Enter zgoal: ";
		cin >> zgoal;
	}
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
		alphaX=atan2(ygoal,xgoal)*180.00/PI;
		if (flag==1){
			fixed_time=a.tm;
			last_alpha=a.rotZ;
			z=a.altd*0.001;
		}
		current_time=a.tm-fixed_time;
		dt=current_time-last_time;
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
		if (flag<=N1){
			v4.push_back(a.rotZ);
			v5.push_back(a.rotX);
			v6.push_back(a.rotY);
			alphaf=sum(v4)/flag;
			bettaf=sum(v5)/flag;
			gamaf=sum(v6)/flag;
		}
		else {
			v4.push_back(a.rotZ);
			v5.push_back(a.rotX);
			v6.push_back(a.rotY);
			v4.erase(v4.begin());
			v5.erase(v5.begin());
			v6.erase(v6.begin());
			alphaf=sum(v4)/N1;
			bettaf=sum(v5)/N1;
			gamaf=sum(v6)/N1;
		}
		vxg=vxf*cos(alphaf*PI/180.00)-vyf*sin(alphaf*PI/180.00);
		vyg=vxf*sin(alphaf*PI/180.00)+vyf*cos(alphaf*PI/180.00);
		x+=vxg*dt/1000.00*pow(10.00,-6.00);
		y+=vyg*dt/1000.00*pow(10.00,-6.00);
		//z+=vzf*dt/1000.00*pow(10.00,-6.00);
		z=a.altd*0.001;
		eps1=(alphaX-alphaf)*PI/180.00;
		eps2=-bettaf*PI/180.00;
		eps3=-gamaf*PI/180.00;
		epsx=xgoal-x;
		epsz=zgoal-z;
		epsy=ygoal-y;
		eps1_integral+=eps1*dt;
		if(dt!=0){
			deps1=(eps1-epslast1)/(dt*pow(10.00,-6.00));
			depsx=(epsx-epsxlast)/(dt*pow(10.00,-6.00));
			depsz=(epsz-epszlast)/(dt*pow(10.00,-6.00));
			deps2=(eps2-epslast2)/(dt*pow(10.00,-6.00));
			deps3=(eps3-epslast3)/(dt*pow(10.00,-6.00));
			w=(alphaf-last_alpha)/(dt*pow(10.00,-6.00));
		}
		goalw=kp1*eps1+ki1*eps1_integral+kd1*deps1;
		if(abs(goalw)>wzmax){
			p.angular.z=sign(goalw)*wzmax;
		}
		else {
			p.angular.z=goalw;
		}
		p.angular.x=kp4*eps2+deps2*kdx;
		p.angular.y=kp4*eps3+deps3*kdy;
		dist=sqrt(pow(epsx,2.0)+pow(epsy,2.0)+pow(epsz,2.0));
		if(abs(alphaX-alphaf)<=eps1max){
			vxgoal=kp2*epsx+kd2*depsx;
			if(dist<=0.04 and flag2==1){
				flag2=0;
			}
			if(flag2==1){
				if(counter==1){
					koef=abs(epsz)/sqrt(pow(epsx,2.0)+pow(epsy,2.0));
				}
				else{
					if(epsx!=0.0 and epsy!=0){
						koef=abs(epsz)/sqrt(pow(epsx,2.0)+pow(epsy,2.0));
					}
				}
				vzgoal=sign(epsz)*abs(vxgoal)*koef;
				if(abs(vxgoal)>=vxmax){
					p.linear.x=sign(epsx)*vxmax;
					if(koef<=0.25){
						p.linear.z=koef*vxmax*sign(epsz);
					}
					else {
						p.linear.z=sign(epsz)*vzmax;
						p.linear.x=vzmax/koef*sign(epsx);
					}
				}
				else if(abs(vzgoal)>=vzmax){
					p.linear.z=vzmax;
					if(koef>=0.25){
						p.linear.x=vzmax/koef*sign(epsx);
					}
					else {
						p.linear.x=vxmax*sign(epsx);
						p.linear.z=koef*vxmax*sign(epsz);
					}
				}
				else {
					p.linear.x=vxgoal;
					p.linear.z=vzgoal;
				}
			}
			else {
				p.linear.z=epsz*kpz+depsz*kdz;
				if(epsx<0.02){
					vygoal=kp22*epsy;
					vxgoal=vygoal*tan(alphaf*PI/180.0);
					if(abs(vygoal)>=vymax){
						vygoal=vymax*sign(epsy);
						p.linear.y=vygoal;
						if(abs(alphaf)<=45.0){
							p.linear.x=vygoal*tan(alphaf*PI/180.00);
						}
						else if((45.0<alphaf<90.0) or (-90.0<alphaf<-45.0)){
							vxgoal=sign(alphaf)*vxmax*sign(epsy);
							p.linear.x=vxgoal;
							p.linear.y=vxgoal/tan(alphaf*PI/180.00);
						}
						else if(abs(alphaf)==90.0){
							p.linear.y=0.0;
							p.linear.x=sign(alphaf)*kp22*epsy;
						}
						else if((90.0<alphaf<135.0) or (-135.0<alphaf<-90.0)){
							vxgoal=sign(alphaf)*vxmax*sign(epsy);
							p.linear.x=vxgoal;
							p.linear.y=vxgoal/tan(alphaf*PI/180.00);
						}
						else{
							p.linear.y=-vygoal;
							p.linear.x=-vygoal*tan(alphaf*PI/180.00);
						}
					}
					else if(abs(vxgoal)>=vxmax){
						vxgoal=vxmax*sign(epsy);
						p.linear.x=vxgoal;
						if((45.0<alphaf<90.0) or (-90.0<alphaf<-45.0)){
							p.linear.y=vxgoal/tan(alphaf*PI/180.00)*sign(alphaf);
						}
						else if(abs(alphaf)==90.0){
							p.linear.y=0.0;
							p.linear.x=vxgoal*sign(alphaf);
						}
						else if((90.0<alphaf<135.0) or (-135.0<alphaf<-90.0)){
							p.linear.y=vxgoal/tan(alphaf*PI/180.00)*sign(alphaf);
						}
					}
					else{
						if((90.0<alphaf) or (alphaf<-90.0)){
							p.linear.y=-vygoal;
							p.linear.x=-vxgoal;
						}
						else if(abs(alphaf)==90.0){
							p.linear.y=0.0;
							p.linear.x=kp22*epsy*sign(alphaf);
						}
						else{
							p.linear.x=vxgoal;
							p.linear.y=vygoal;
						}
					}
				}
				else {
					p.linear.x=epsx*kpx;
				}
			}
			counter+=1;
		}
		sheet->writeNum(flag,0,current_time*pow(10.00,-6.00),format);
		sheet->writeNum(flag,1,eps1*180/PI,format);
		sheet->writeNum(flag,2,epsx,format);
		sheet->writeNum(flag,3,epsy,format);
		sheet->writeNum(flag,4,epsz,format);
		sheet->writeNum(flag,5,dist,format);
		sheet->writeNum(flag,6,depsx,format);
		sheet->writeNum(flag,7,epsx,format);
		sheet->writeNum(flag,8,epsxlast,format);
		sheet->writeNum(flag,9,a.rotX,format);
		sheet->writeNum(flag,10,a.rotY,format);
		sheet->writeNum(flag,11,bettaf,format);
		sheet->writeNum(flag,12,gamaf,format);
		sheet->writeNum(flag,13,x,format);
		sheet->writeNum(flag,14,y,format);
		sheet->writeNum(flag,15,z,format);
		flag+=1;
		last_alpha=alphaf;
		last_time=current_time;
		epslast1=eps1;
		epslast2=eps2;
		epslast3=eps3;
		epsxlast=epsx;
		epszlast=epsz;
		pub_.publish(p);
		sleep(0.005);
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
	cout << "Enter xgoal: ";
	cin >> xgoal;
	cout << "\n" << "Enter ygoal: ";
	cin >> ygoal;
	cout << "\n" << "Enter zygoal: ";
	cin >> zgoal;
	SubscribeAndPublish SAPObject;
	signal(SIGINT, mySigintHandler);
	ros::spin();
	return 0;
}
