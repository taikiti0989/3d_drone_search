#include "ManualControl.h"
#include "Patroller.h"
#include "StateTracker.h"
#include <cmath>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <kl_evaluation/kl_suzu.h>
#include <kl_evaluation/kl_suzuki.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <bebop_test/flag.h>
using namespace std;

const int frameMiddleX=428;
//const int frameMiddleY=240;
extern string file_data_odom_z;
extern ofstream fout_data_odom_z;
int ii;
extern string file_data_way_marker_x;
extern ofstream fout_data_way_marker_x;
extern string file_data_way_marker_y;
extern ofstream fout_data_way_marker_y;
extern string file_data_way_marker_z;
extern ofstream fout_data_way_marker_z;
extern string file_data_way_marker_yaw;
extern ofstream fout_data_way_marker_yaw;
extern string file_data_way_kl;
extern ofstream fout_data_way_kl;
extern string file_data_marker_x;
extern ofstream fout_data_marker_x;
extern string file_data_marker_y;
extern ofstream fout_data_marker_y;
extern string file_data_marker_z;
extern ofstream fout_data_marker_z;
extern string file_data_marker_yaw;
extern ofstream fout_data_marker_yaw;
extern string file_data_kl;
extern ofstream fout_data_kl;
extern string file_data_time;
extern ofstream fout_data_time;
extern string file_data_way_time;
extern ofstream fout_data_way_time;

extern string file_data_map_x;
extern ofstream fout_data_map_x;
extern string file_data_map_y;
extern ofstream fout_data_map_y;
extern string file_data_map_z;
extern ofstream fout_data_map_z;
extern string file_data_map_kl;
extern ofstream fout_data_map_kl;
extern string file_data_target_x;
extern ofstream fout_data_target_x;
extern string file_data_target_y;
extern ofstream fout_data_target_y;
extern string file_data_target_z;
extern ofstream fout_data_target_z;

int end_flag=0;
extern double ros_begin;
double ros_duration;
extern geometry_msgs::Twist feed;
extern geometry_msgs::Twist feed_red;
extern double kl_s;
double bkl_s;
double optimal_x;
double optimal_y;
double optimal_z;
double kl_best=100;
extern double marker_x;
extern double marker_y;
extern double marker_z;
extern double marker_yaw;
extern double marker_yaw2;
extern double marker_pitch;
double best_score=100;
extern double odom_z;
extern double slam_x;
extern double slam_y;
extern double slam_z;
extern double slam_yaw;
extern double slam_roll;
int pso_flag=0;
double ab_y;
double ab_z;
int flag=0;
int kl_flag=0;
int rflag=0;
int tflag=0;
int num=0;
double pi=3.1415;
double r_suzu;//半径
double theta;
double way_x;
double way_y;
double way_z=1.0;
double target_x;
double target_y;
double target_z;
double target_theta;
double dif;
double theta_suzu;
double time1;
double time0;
double time_rec;
double time2;
double dtime;
double V=0.06;//0.05;
extern kl_evaluation::kl_suzu score;
extern kl_evaluation::kl_suzuki scores;
extern kl_evaluation::kl_suzuki scores_pub;
int j = 0;
extern kl_evaluation::kl_suzu semi_opt;
extern int flag_semi;
extern int flag_gau;
extern bebop_test::flag cv_flag;
double pso_x;
double pso_y;
double R;
double Rrate;
double best_x;
double best_y;
double best_z;
int ran_x;
int ran_y;
int ran_z;
double ran_x2;
double ran_y2;
double ran_z2;
int pso_flag2=0;
int new_flag=0;
double yoko=3.0;
double tate=1.5;
int num1;
vector<double> data;
//vector<int> pin;
int pin[10]={0,0,0,0,0,0,0,0,0,0};
int resul=0;
///////////////////////////pso/////////////////////////////
struct Point{
	double x;
	double y;
};
// double pso_eval(double x, double y,double z){
// 	z=x*x+y*y;
// 	return z;
// }
// double up_posi(double x,double y,double vx,double vy,double new_x,double new_y){
// 	new_x=x+vx;
// 	new_y=y+vy;
// 	return (new_x,new_y);
// }
// double up_vel(double x,double y,double vx,double vy,Point p,double new_vx,double new_vy){
// 	srand(time(NULL));
// 	double w0=0.5;
// 	double w1=(double)rand()/RAND_MAX;
// 	new_vx=w0*vx+w1*(p.x-x);
// 	new_vy=w0*vy+w1*(p.y-y);
// 	return (new_vx,new_vy);
// }
double sigmoid(double gain, double x) {
    return 1.0 / (1.0 + exp(gain * (x-0.5)));
}


// double pso_score=100;
// double gbest=100;
// Point l;//ローカルベスト　ガウス過程から
// Point p;//パーソナルベスト　移動軌跡から
// double time_begin;
// double time_now;
// double new_vx,new_vy;
// double new_x,new_y;
// int limit=60;

// 	//粒子の数1 粒子の初期化 位置
// 	//double now_x,now_y;
// 	double now_x=2.8;//slam_x;
// 	double now_y=4.3;//slam_y;
// 	//cout<<"now_x: "<<now_x<<endl;
// 	//cout<<"now_y: "<<now_y<<endl;	
// 	//double now_vx,now_vy;
// 	double now_vx=0;
// 	double now_vy=0;
	//cout<<"now_vx: "<<now_vx<<endl;
	//cout<<"now_vy: "<<now_vy<<endl;

//////////////////////////pso///////////////////////////////
Patroller* patroller;

Patroller::Patroller() {}

Patroller::~Patroller() {
	destroy();
}

void Patroller::destroy() {}

void Patroller::checkState() {
	// switch statement in case we need to check signage
	// ROS_INFO( "OFF: x: %f Y: %f", fabs(stats->getOdom()->pose.pose.position.x - current_state.start.x), fabs(stats->getOdom()->pose.pose.position.y - current_state.start.y) );
	switch(current_state.direction) {
	case FORWARD:
	case BACKWARD:
		if(fabs(stats->getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) nextState();
		break;

	case RIGHT:
	case LEFT:
		if(fabs(stats->getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) nextState();
		break;
	}
}

void Patroller::nextState() {
	current_state.start = stats->getOdom()->pose.pose.position;

	if(current_state.direction == RIGHT || current_state.direction == LEFT) current_state.distance = current_state.distance + spacing;
	else current_state.distance = current_state.distance;
	current_state.direction = (current_state.direction + 1) % 4;

	if(current_state.direction == BACKWARD && current_state.distance > radius) patrolling = false;

	// ROS_INFO("SWITCHING TO STATE: DIR: %d DIS: %f", current_state.direction, current_state.distance);
}

// simple reflex function that responds to current altitude of bebop and state information
void Patroller::patrol() {
	if(!patrolling) return;

	double off = 0;

	// ROS_INFO("PATROLLING WITH STATE: DIR: %d DIS: %f", current_state.direction, current_state.distance);

	checkState();
	geometry_msgs::Twist vel;
	speed = control->getSpeed();
	switch(current_state.direction) {
	case FORWARD:

		// ROS_INFO("EXECUTING FORWARD");
		vel.linear.x = speed;
		off = (stats->getOdom()->pose.pose.position.y - current_state.start.y);
		vel.linear.y = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case RIGHT:

		// ROS_INFO("EXECUTING RIGHT");
		vel.linear.y = -speed;
		off = (stats->getOdom()->pose.pose.position.x - current_state.start.x);
		vel.linear.x = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case BACKWARD:

		// ROS_INFO("EXECUTING BACKWARD");
		vel.linear.x = -speed;
		off = (stats->getOdom()->pose.pose.position.y - current_state.start.y);
		vel.linear.y = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case LEFT:

		// ROS_INFO("EXECUTING LEFT");
		vel.linear.y = speed;
		off = (stats->getOdom()->pose.pose.position.x - current_state.start.x);
		vel.linear.x = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

//////Draw a circle with constant linear velocity///////////////////////
////by lan/////
	case CIRCLE:
fout_data_marker_x<<slam_x<<endl;
fout_data_marker_y<<slam_y<<endl;
fout_data_marker_z<<slam_z<<endl;
fout_data_marker_yaw<<slam_roll<<endl;
fout_data_kl<<kl_s<<endl;
time_rec=ros::Time::now().toSec();
fout_data_time<<dtime<<endl;
// cout<<"semi_kl: "<<semi_opt.kl_score<<endl;
// if(bkl_s=!semi_opt.kl_score){
// 	new_flag=0;
// 	cout<<"nnnnnnnnnnnnnnnnnnnnnn"<<endl;
// }
// bkl_s=semi_opt.kl_score;
// if(flag==2){
// 	if(new_flag==0){

// 	}
// }
if(flag==0){
	if(feed.angular.z==0){
		//記録
		fout_data_way_marker_x<<slam_x<<endl;
		fout_data_way_marker_y<<slam_y<<endl;
		fout_data_way_marker_z<<slam_z<<endl;
		fout_data_way_marker_yaw<<slam_roll<<endl;
		fout_data_way_kl<<kl_s<<endl;
		fout_data_way_time<<dtime<<endl;
		num = num + 1;
		//scores格納
		score.num = num;
		score.kl_score = kl_s;
		score.kl_x = slam_x;
		score.kl_y = slam_y;
		score.kl_z = slam_z;
		scores.KL.push_back(score);
		flag=10;
		time0=ros::Time::now().toSec();
	}
}
if(flag==10){
	srand(time(NULL));
	ran_x=rand()%100;
	ran_y=rand()%100;
	ran_z=rand()%100;
	cout<<"ran_x: "<<ran_x<<endl;
	cout<<"ran_y: "<<ran_y<<endl;
	cout<<"ran_z: "<<ran_z<<endl;
	ran_x2=ran_x;
	ran_y2=ran_y;
	ran_z2=ran_z;
	//way_x=ran_x2*3/100-1.5;
	way_x=-(yoko/2.0)+(yoko/3.0)*ran_x2/100;
	way_y=ran_y2*tate/100-(tate/2);
	way_z=0.8+ran_z2/100;
	cout<<"way_x: "<<way_x<<endl;
	cout<<"way_y: "<<way_y<<endl;
	cout<<"way_z: "<<way_z<<endl;
	flag=20;
	time1=ros::Time::now().toSec();
}
if(flag==11){
	srand(time(NULL));
	ran_x=rand()%100;
	ran_y=rand()%100;
	ran_z=rand()%100;
	cout<<"ran_x: "<<ran_x<<endl;
	cout<<"ran_y: "<<ran_y<<endl;
	cout<<"ran_z: "<<ran_z<<endl;
	ran_x2=ran_x;
	ran_y2=ran_y;
	ran_z2=ran_z;
	//way_x=ran_x2*3/100-1.5;
	way_x=-(yoko/6.0)+(yoko/3.0)*ran_x2/100;
	way_y=ran_y2*tate/100-(tate/2);
	way_z=0.8+ran_z2/100;
	cout<<"way_x: "<<way_x<<endl;
	cout<<"way_y: "<<way_y<<endl;
	cout<<"way_z: "<<way_z<<endl;
	flag=21;
	time1=ros::Time::now().toSec();
}
if(flag==12){
	srand(time(NULL));
	ran_x=rand()%100;
	ran_y=rand()%100;
	ran_z=rand()%100;
	cout<<"ran_x: "<<ran_x<<endl;
	cout<<"ran_y: "<<ran_y<<endl;
	cout<<"ran_z: "<<ran_z<<endl;
	ran_x2=ran_x;
	ran_y2=ran_y;
	ran_z2=ran_z;
	//way_x=ran_x2*3/100-1.5;
	way_x=(yoko/6.0)+(yoko/3.0)*ran_x2/100;
	way_y=ran_y2*tate/100-(tate/2);
	way_z=0.8+ran_z2/100;
	cout<<"way_x: "<<way_x<<endl;
	cout<<"way_y: "<<way_y<<endl;
	cout<<"way_z: "<<way_z<<endl;
	flag=22;
	time1=ros::Time::now().toSec();
}

if(flag==20){
	time2=ros::Time::now().toSec()-time1;
	cout<<"time2: "<<time2<<endl;
	target_x = way_x - slam_x;
	target_y = way_y - slam_y;
	target_z = way_z - slam_z;
	ab_y = fabs(way_x - slam_x);
	ab_z = fabs(way_y - slam_y);
	dif = sqrt(target_x * target_x + target_y * target_y + target_z * target_z);
	target_theta = atan(ab_y/ab_z);
	///////1///////
	if(target_x<0 && target_y>0){
		double a = target_theta - slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("1");
	}
	//////2////////
	if(target_x>0 && target_y>0){
		double a = target_theta + slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("2");
	}
	//////3////////
	if(target_x>0 && target_y<0){
		double a = target_theta - slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("3");
	}
	if(target_x<0 && target_y<0){
		double a = target_theta + slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("4");
	}
	if(dif<0.1){
		//記録
		fout_data_way_marker_x<<slam_x<<endl;
		fout_data_way_marker_y<<slam_y<<endl;
		fout_data_way_marker_z<<slam_z<<endl;
		fout_data_way_marker_yaw<<slam_roll<<endl;
		fout_data_way_kl<<kl_s<<endl;
		fout_data_way_time<<dtime<<endl;
		num = num + 1;
		//scores格納
		score.num = num;
		score.kl_score = kl_s;
		score.kl_x = slam_x;
		score.kl_y = slam_y;
		score.kl_z = slam_z;
		scores.KL.push_back(score);
		if(num1==1){
			//kl_flag=1;
			flag=11;
			num1=0;
		}
		else{
			num1=num1+1;
			flag=10;
		}
	}
	// if(time2>3){
	// 	fout_data_way_marker_x<<slam_x<<endl;
	// 	fout_data_way_marker_y<<slam_y<<endl;
	// 	fout_data_way_marker_z<<slam_z<<endl;
	// 	fout_data_way_marker_yaw<<slam_roll<<endl;
	// 	fout_data_way_kl<<kl_s<<endl;
	// 	num = num + 1;
	// 	score.num = num;
	// 	score.kl_score = kl_s;
	// 	score.kl_x = slam_x;
	// 	score.kl_y = slam_y;
	// 	score.kl_z = slam_z;
	// 	scores.KL.push_back(score);
	// 	time1=ros::Time::now().toSec();
	// 	ROS_INFO("ttttttttttttttttt");
	// }
}
if(flag==21){
	time2=ros::Time::now().toSec()-time1;
	cout<<"time2: "<<time2<<endl;
	target_x = way_x - slam_x;
	target_y = way_y - slam_y;
	target_z = way_z - slam_z;
	ab_y = fabs(way_x - slam_x);
	ab_z = fabs(way_y - slam_y);
	dif = sqrt(target_x * target_x + target_y * target_y + target_z * target_z);
	target_theta = atan(ab_y/ab_z);
	///////1///////
	if(target_x<0 && target_y>0){
		double a = target_theta - slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("1");
	}
	//////2////////
	if(target_x>0 && target_y>0){
		double a = target_theta + slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("2");
	}
	//////3////////
	if(target_x>0 && target_y<0){
		double a = target_theta - slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("3");
	}
	if(target_x<0 && target_y<0){
		double a = target_theta + slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("4");
	}
	if(dif<0.1){
		//記録
		fout_data_way_marker_x<<slam_x<<endl;
		fout_data_way_marker_y<<slam_y<<endl;
		fout_data_way_marker_z<<slam_z<<endl;
		fout_data_way_marker_yaw<<slam_roll<<endl;
		fout_data_way_kl<<kl_s<<endl;
		fout_data_way_time<<dtime<<endl;
		num = num + 1;
		//scores格納
		score.num = num;
		score.kl_score = kl_s;
		score.kl_x = slam_x;
		score.kl_y = slam_y;
		score.kl_z = slam_z;
		scores.KL.push_back(score);
		if(num1==1){
			//kl_flag=1;
			flag=12;
			num1=0;
		}
		else{
			num1=num1+1;
			flag=11;
		}
	}
}
if(flag==22){
	time2=ros::Time::now().toSec()-time1;
	cout<<"time2: "<<time2<<endl;
	target_x = way_x - slam_x;
	target_y = way_y - slam_y;
	target_z = way_z - slam_z;	
	ab_y = fabs(way_x - slam_x);
	ab_z = fabs(way_y - slam_y);
	dif = sqrt(target_x * target_x + target_y * target_y + target_z * target_z);
	target_theta = atan(ab_y/ab_z);
	///////1///////
	if(target_x<0 && target_y>0){
		double a = target_theta - slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("1");
	}
	//////2////////
	if(target_x>0 && target_y>0){
		double a = target_theta + slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("2");
	}
	//////3////////
	if(target_x>0 && target_y<0){
		double a = target_theta - slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("3");
	}
	if(target_x<0 && target_y<0){
		double a = target_theta + slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("4");
	}
	if(dif<0.1){
		//記録
		fout_data_way_marker_x<<slam_x<<endl;
		fout_data_way_marker_y<<slam_y<<endl;
		fout_data_way_marker_z<<slam_z<<endl;
		fout_data_way_marker_yaw<<slam_roll<<endl;
		fout_data_way_kl<<kl_s<<endl;
		fout_data_way_time<<dtime<<endl;
		num = num + 1;
		//scores格納
		score.num = num;
		score.kl_score = kl_s;
		score.kl_x = slam_x;
		score.kl_y = slam_y;
		score.kl_z = slam_z;
		scores.KL.push_back(score);
		if(num1==1){
			kl_flag=1;
			flag=30;
		}
		else{
			num1=num1+1;
			flag=12;
		}
	}
}
if(flag==30){
	if(pso_flag==1){
		if(new_flag==0){
			Rrate=0.5;//sigmoid(10,semi_opt.kl_z);
			R=Rrate;
			cout<<"Rrate"<<R<<endl;
			srand(time(NULL));
			ran_x=rand()%100;
			ran_y=rand()%100;
			ran_z=rand()%100;
			cout<<"ran_x: "<<ran_x<<endl;
			cout<<"ran_y: "<<ran_y<<endl;
			cout<<"ran_z: "<<ran_z<<endl;
			ran_x2=ran_x;
			ran_y2=ran_y;
			ran_z2=ran_z;
			way_x=semi_opt.kl_x+ran_x2/100*R-R/2;
			way_y=semi_opt.kl_y+ran_y2/100*R-R/2;
			way_z=semi_opt.kl_z+ran_z2/100*0.2-0.2/2;
			if(way_x>(yoko/2)){
				way_x=yoko/2;
			}
			if(way_x<-(yoko/2)){
				way_x=-yoko/2;
			}
			if(way_y>(tate/2)){
				way_y=tate/2;
			}
			if(way_y<-(tate/2)){
				way_y=-tate/2;
			}
			if(way_z>1.8){
				way_z=1.8;
			}
			if(way_z<0.8){
				way_z=0.8;
			}
			cout<<"way_x: "<<way_x<<endl;
			cout<<"way_y: "<<way_y<<endl;
			cout<<"way_z: "<<way_z<<endl;
			fout_data_map_x<<semi_opt.kl_x<<endl;
			fout_data_map_y<<semi_opt.kl_y<<endl;
			fout_data_map_z<<semi_opt.kl_z<<endl;
			fout_data_map_kl<<semi_opt.kl_score<<endl;
			fout_data_target_x<<way_x<<endl;
			fout_data_target_y<<way_y<<endl;
			fout_data_target_z<<way_z<<endl;
			pso_flag2=1;
			new_flag=1;
		}
	}
	if(pso_flag2==1){
		time2=ros::Time::now().toSec()-time1;
		cout<<"time: "<<time2<<endl;
		target_x = way_x - slam_x;
		target_y = way_y - slam_y;
		target_z = way_z - slam_z;
		ab_y = fabs(way_x - slam_x);
		ab_z = fabs(way_y - slam_y);
		dif = sqrt(target_x * target_x + target_y * target_y + target_z * target_z);
		target_theta = atan(ab_y/ab_z);
		///////1///////
		if(target_x<0 && target_y>0){
			double a = target_theta - slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			ROS_INFO("1");
		}
		//////2////////
		if(target_x>0 && target_y>0){
			double a = target_theta + slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			ROS_INFO("2");
		}
		//////3////////
		if(target_x>0 && target_y<0){
			double a = target_theta - slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			ROS_INFO("3");
		}
		if(target_x<0 && target_y<0){
			double a = target_theta + slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			ROS_INFO("4");
		}
		if(dif<0.1){
			fout_data_way_marker_x<<slam_x<<endl;
			fout_data_way_marker_y<<slam_y<<endl;
			fout_data_way_marker_z<<slam_z<<endl;
			fout_data_way_marker_yaw<<slam_roll<<endl;
			fout_data_way_kl<<kl_s<<endl;
			fout_data_way_time<<dtime<<endl;
			num = num + 1;
			score.num = num;
			score.kl_score = kl_s;
			score.kl_x = slam_x;
			score.kl_y = slam_y;
			score.kl_z = slam_z;
			scores.KL.push_back(score);
			new_flag=0;
			pso_flag2=0;
		}
		if(time2>3.0){
			fout_data_way_marker_x<<slam_x<<endl;
			fout_data_way_marker_y<<slam_y<<endl;
			fout_data_way_marker_z<<slam_z<<endl;
			fout_data_way_marker_yaw<<slam_roll<<endl;
			fout_data_way_kl<<kl_s<<endl;
			fout_data_way_time<<dtime<<endl;
			num = num + 1;
			score.num = num;
			score.kl_score = kl_s;
			score.kl_x = slam_x;
			score.kl_y = slam_y;
			score.kl_z = slam_z;
			scores.KL.push_back(score);
			time1=ros::Time::now().toSec();
			ROS_INFO("ttttttttttttttttt");
		}
	}
}
if(best_score>kl_s){
	best_score=kl_s;
	best_x=slam_x;
	best_y=slam_y;
	best_z=slam_z;
}
data.push_back(kl_s);
if(data.size()>=10){
	ROS_INFO("101010101010");
	int l=data.size();
	for(int i=1;i<=10;i++){
		if(data[l-i]<1){
			ROS_INFO("111111111");
			pin[i]=1;
		}
		else{
			pin[i]=0;
			ROS_INFO("000000000");
		}
	}
	resul=pin[0]+pin[1]+pin[2]+pin[3]+pin[4]+pin[5]+pin[6]+pin[7]+pin[8]+pin[9];
	if(resul==10){
		flag=6;
 	}
}
cout<<"personal best: "<<best_score<<endl;
cout<<"best_x: "<<best_x<<endl;
cout<<"best_y: "<<best_y<<endl;
cout<<"best_z: "<<best_z<<endl;



/*if(flag==3){
	dtime=ros::Time::now().toSec()-time0;
	if(pso_flag=1){
		//if(pso_flag2==0)
		{
			if(new_flag==0){
				time0=ros::Time::now().toSec();
				cout<<"0000000"<<endl;
				Rrate=sigmoid(10,semi_opt.kl_z);
			R=Rrate*3;
			cout<<"Rrate"<<R<<endl;
			srand(time(NULL));
			ran_x=rand()%100;
			ran_y=rand()%100;
			cout<<"ran_x: "<<ran_x<<endl;
			cout<<"ran_y: "<<ran_y<<endl;
			ran_x2=ran_x;
			ran_y2=ran_y;
			way_x=semi_opt.kl_x+ran_x2/100*R-R/2;
			way_y=semi_opt.kl_y+ran_y2/100*R-R/2;
			cout<<"way_x: "<<way_x<<endl;
			cout<<"way_y: "<<way_y<<endl;
			double dx,dy;
			dx=fabs(way_x);
			dy=fabs(way_y);
			cout<<"dx: "<<dx<<endl;
			cout<<"dy: "<<dy<<endl;
			// if(dx>1.5){
			// 	cout<<"///////////////////////////////////////////////"<<endl;
			// }
			// else if(dy>0.75){
			// 	cout<<"///////////////////////////////////////////////"<<endl;
			// }
			if(way_x>(yoko/2)){
				way_x=yoko/2;
			}
			if(way_x<-(yoko/2)){
				way_x=-yoko/2;
			}
			if(way_y>(tate/2)){
				way_y=tate/2;
			}
			if(way_y<-(tate/2)){
				way_y=-tate/2;
			}
			//else
			{
				//cout<<"33333333"<<endl;
				pso_flag2=1;
				time1=ros::Time::now().toSec();
				new_flag=1;
			}
			}
		}
	}
	if(pso_flag2==1){
		time2=ros::Time::now().toSec()-time1;
		cout<<"time: "<<time2<<endl;
		target_x = way_x - slam_x;
		target_y = way_y - slam_y;
		ab_y = fabs(way_x - slam_x);
		ab_z = fabs(way_y - slam_y);
		dif = sqrt(target_x * target_x + target_y * target_y);
		target_theta = atan(ab_y/ab_z);
		///////1///////
		if(target_x<0 && target_y>0){
			double a = target_theta - slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			ROS_INFO("1");
		}
		//////2////////
		if(target_x>0 && target_y>0){
			double a = target_theta + slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			ROS_INFO("2");
		}
		//////3////////
		if(target_x>0 && target_y<0){
			double a = target_theta - slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			ROS_INFO("3");
		}
		if(target_x<0 && target_y<0){
			double a = target_theta + slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			ROS_INFO("4");
		}
		if(dif<0.1){
		// 	//記録
		// 	fout_data_way_marker_x<<slam_x<<endl;
		// 	fout_data_way_marker_y<<slam_y<<endl;
		// 	fout_data_way_marker_z<<slam_z<<endl;
		// 	fout_data_way_marker_yaw<<slam_roll<<endl;
		// 	fout_data_way_kl<<kl_s<<endl;
		// 	num = num + 1;
		// 	//scores格納
		// 	score.num = num;
		// 	score.kl_score = kl_s;
		// 	score.kl_x = slam_x;
		// 	score.kl_y = slam_y;
		// 	score.kl_z = slam_z;
		// 	scores.KL.push_back(score);
			flag=1;
		}
		if(time2>3){
			fout_data_way_marker_x<<slam_x<<endl;
			fout_data_way_marker_y<<slam_y<<endl;
			fout_data_way_marker_z<<slam_z<<endl;
			fout_data_way_marker_yaw<<slam_roll<<endl;
			fout_data_way_kl<<kl_s<<endl;
			num = num + 1;
			score.num = num;
			score.kl_score = kl_s;
			score.kl_x = slam_x;
			score.kl_y = slam_y;
			score.kl_z = slam_z;
			scores.KL.push_back(score);
			time1=ros::Time::now().toSec();
			ROS_INFO("ttttttttttttttttt");
		}
	}
}*/

dtime=ros::Time::now().toSec()-time0;
if(dtime>10000){
	dtime=0;
}
cout<<"dtime: "<<dtime<<endl;
if(dtime>100){
	if(end_flag==0){
		flag=7;
		end_flag=1;
	}
}
if(best_score<1){
	////////////////////////////flag=6;
}
if(flag==6){
	cout<<"end~~~~~~~~~~~~~~~"<<endl;
}
if(flag==7){
	way_x=best_x;
	way_y=best_y;
	way_z=best_z;
		target_x = way_x - slam_x;
		target_y = way_y - slam_y;
		target_z = way_z - slam_z;
		ab_y = fabs(way_x - slam_x);
		ab_z = fabs(way_y - slam_y);
		dif = sqrt(target_x * target_x + target_y * target_y+ target_z * target_z);
		target_theta = atan(ab_y/ab_z);
		///////1///////
		if(target_x<0 && target_y>0){
			double a = target_theta - slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			ROS_INFO("1");
		}
		//////2////////
		if(target_x>0 && target_y>0){
			double a = target_theta + slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			ROS_INFO("2");
		}
		//////3////////
		if(target_x>0 && target_y<0){
			double a = target_theta - slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			ROS_INFO("3");
		}
		if(target_x<0 && target_y<0){
			double a = target_theta + slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			ROS_INFO("4");
		}
		if(dif<0.1){
			ROS_INFO("666666666666666666666666666666");
			flag=6;
		}
}
/*
if(flag==0){
	time1=ros::Time::now().toSec();
	srand(time(NULL));
	ran_x = rand()%100;
	ran_y = rand()%100;
	cout<<"ran_x"<<ran_x<<endl;
	cout<<"ran_y"<<ran_y<<endl;
	pso_x = ran_x*0.6/100-0.3;
	pso_y = ran_y*0.6/100-0.3;
	cout<<"pso_x"<<pso_x<<endl;
	cout<<"pso_y"<<pso_y<<endl;
	way_x=slam_x+pso_x;
	way_y=slam_y+pso_y;
	//flag=1;
	cv_flag.num=1;
	R=sigmoid(10,);
	cout<<"rrrr"<<R<<endl;
}
if(flag==6){
	srand(time(NULL));
	// pso_x = rand()%100;
	// pso_y = rand()%100;
	pso_x = pso_x/100;
	pso_y = pso_y/100;
	cout<<"pso_x00"<<pso_x<<endl;
	cout<<"pso_y00"<<pso_y<<endl;
	pso_x = pso_x*0.6-0.3;
	pso_y = pso_y*0.6-0.3;
	cout<<"pso_x11"<<pso_x<<endl;
	cout<<"pso_y11"<<pso_y<<endl;
	way_x=slam_x+pso_x;
	way_y=slam_y+pso_y;
	flag=1;
	cv_flag.num=1;
}
if(flag==1){
	time2=ros::Time::now().toSec()-time1;
	cout<<"time"<<time2<<endl;
	target_x = way_x - slam_x;
	target_y = way_y - slam_y;
	ab_y = fabs(way_x - slam_x);
	ab_z = fabs(way_y - slam_y);
	dif = sqrt(target_x * target_x + target_y * target_y);
	target_theta = atan(ab_y/ab_z);
	///////1///////
	if(target_x<0 && target_y>0){
		double a = target_theta - slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("1");
	}
	//////2////////
	if(target_x>0 && target_y>0){
		double a = target_theta + slam_roll;
		vel.linear.x = -V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("2");
	}
	//////3////////
	if(target_x>0 && target_y<0){
		double a = target_theta - slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = -V*dif*sin(a);
		ROS_INFO("3");
	}
	if(target_x<0 && target_y<0){
		double a = target_theta + slam_roll;
		vel.linear.x = V*dif*cos(a);
		vel.linear.y = V*dif*sin(a);
		ROS_INFO("4");
	}
	if(dif<0.1){
		fout_data_way_marker_x<<slam_x<<endl;
		fout_data_way_marker_y<<slam_y<<endl;
		fout_data_way_marker_z<<slam_z<<endl;
		fout_data_way_marker_yaw<<slam_roll<<endl;
		fout_data_way_kl<<kl_s<<endl;
		flag=6;
		cout<<"222222222222"<<endl;
		cout<<"222222222222"<<endl;
		cout<<"222222222222"<<endl;
		cout<<"222222222222"<<endl;
		cout<<"222222222222"<<endl;
	}
	if(best_score > kl_s){
		best_score=kl_s;
		p.x=slam_x;
		p.y=slam_y;
	}
	if(best_score<1){
		cv_flag.num=3;
		flag=2;
	}
	if(time2>60){
		cv_flag.num=3;
		flag=2;
	}
}
if(flag==2){
	cout<<"1111111111111"<<endl;
}
*/
/////////////////////////////////////////////////


/*
//if(marker_x!=0)
//{
	//flag0 waypointまで移動
	if(flag == 0){
		cv_flag.num=1;
		ROS_INFO("flag0");
		way_x = -1.5;
		way_y = 0.5;
		target_x = way_x - slam_x;
		target_y = way_y - slam_y;
		ab_y = fabs(way_x - slam_x);
		ab_z = fabs(way_y - slam_y);
		dif = sqrt(target_x * target_x + target_y * target_y);
		target_theta = atan(ab_y/ab_z);
		///////1///////
		if(target_x<0 && target_y>0){
			double a = target_theta - slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("1");
		}
		//////2////////
		if(target_x>0 && target_y>0){
			double a = target_theta + slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("2");
		}
		//////3////////
		if(target_x>0 && target_y<0){
			double a = target_theta - slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("3");
		}
		if(target_x<0 && target_y<0){
			double a = target_theta + slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("4");
		}
		if(dif<0.1){
			flag=1;
			time1=ros::Time::now().toSec();
		}
		//cout<<"vel.linear.x: "<<vel.linear.x<<endl;
		//cout<<"vel.linear.y: "<<vel.linear.y<<endl;
	}
	////////////flag1 waypoint 更新
	if(flag == 1){
		ROS_INFO("flag1");
		vel.linear.y = 0;
		vel.linear.x = 0;
		target_x = way_x - slam_x;
		target_y = way_y - slam_y;
		dif = sqrt(target_x * target_x + target_y * target_y);
		dtime=ros::Time::now().toSec()-time1;
		cout<<"dtime: "<<dtime<<endl;
		if(feed.angular.z==0)
		{
			// cout<<"kkkkkkkkkkkkkkkkkkkkkkk"<<endl;
			// if(dif<0.1)
			{
				//for(int i=0;i<4;i++){
				fout_data_way_marker_x<<slam_x<<endl;
				fout_data_way_marker_y<<slam_y<<endl;
				fout_data_way_marker_z<<slam_z<<endl;
				fout_data_way_marker_yaw<<slam_roll<<endl;
				fout_data_way_kl<<kl_s<<endl;
				num = num + 1;
				score.num = j;
				score.kl_score = kl_s;
				score.kl_x = slam_x;
				score.kl_y = slam_y;
				score.kl_z = slam_z;
				pso_score=kl_s;
				// cout<<"score: "<<pso_score<<endl;
				// cout<<"px: "<<p.x<<endl;
				// cout<<"py: "<<p.y<<endl;
				// cout<<"personal best: "<<gbest<<endl;
				//score.kl_time = ros::Time::now().toSec()-time1;
				scores.KL.push_back(score);
				j = j++;
				//}
				if(num > 4){
					num = 0;
					if(tflag < 3){//waypointの数-1
						if(rflag % 2 == 0){
							way_x = way_x + 1.0;
							tflag = tflag + 1;
						}
						else{
							way_x = way_x - 1.0;
							tflag = tflag + 1;
						}
						flag=2;
					}
					else{
						if(rflag < 1){
							way_y = way_y - 1.0;
							tflag = 0;
							rflag = rflag + 1;
							flag=2;
						}
						else{
							tflag = 0;
							rflag = 0;
							flag = 3;//3
						}
					}
				}
				// else{
				// 	flag = 2;
				// }
			}
			//  else
			//  {
			//  	flag = 2;
			//  }
		}
		if(gbest>kl_s){
			gbest=kl_s;
			p.x=slam_x;
			p.y=slam_y;
		}
	}
	else if(flag == 2){
		ROS_INFO("flag2");
		target_x = way_x - slam_x;
		target_y = way_y - slam_y;
		ab_y = fabs(way_x - slam_x);
		ab_z = fabs(way_y - slam_y);
		dif = sqrt(target_x * target_x + target_y * target_y);
		target_theta = atan(ab_y/ab_z);
		///////1///////
		if(target_x<0 && target_y>0){
			double a = target_theta - slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("1");
		}
		//////2////////
		if(target_x>0 && target_y>0){
			double a = target_theta + slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("2");
		}
		//////3////////
		if(target_x>0 && target_y<0){
			double a = target_theta - slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("3");
		}
		if(target_x<0 && target_y<0){
			double a = target_theta + slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			// ROS_INFO("aaaaaaaaaa: %f",a );
			ROS_INFO("4");
		}
		if(dif<0.1){
			flag=1;
			time1=ros::Time::now().toSec();
		}
		if(gbest>kl_s){
			gbest=kl_s;
			p.x=slam_x;
			p.y=slam_y;
		}
	}
	// else if(flag == 3){
	// 	ROS_INFO("flag3");
	// 	kl_flag = 1;
	// 	if(flag_semi == 1){
	// 		ROS_INFO("semi_flag 1");
	// 		way_x = semi_opt.kl_x;
	// 		way_y = semi_opt.kl_y;
	// 		cout<<"score"<<semi_opt.kl_score<<endl;
    //         cout<<"xxxx"<<semi_opt.kl_x<<endl;
    //         cout<<"yyyy"<<semi_opt.kl_y<<endl;
    //         cout<<"zzzz"<<semi_opt.kl_z<<endl;
	// 		target_x = way_x - slam_x;
	// 		target_y = way_y - slam_y;
	// 		ab_y = fabs(way_x - slam_x);
	// 		ab_z = fabs(way_y - slam_y);
	// 		dif = sqrt(target_x*target_x+target_y*target_y);
	// 		target_theta = atan(ab_y/ab_z);
	// 		///////1///////
	// 		if(target_x < 0 && target_y > 0){
	// 			double a = target_theta - slam_roll;
	// 			vel.linear.x = -V*dif*cos(a);
	// 			vel.linear.y = V*dif*sin(a);
	// 		}
	// 		//////2////////
	// 		if(target_x > 0 && target_y > 0){
	// 			double a = target_theta + slam_roll;
	// 			vel.linear.x = -V*dif*cos(a);
	// 			vel.linear.y = -V*dif*sin(a);
	// 		}
	// 		//////3////////
	// 		if(target_x > 0 && target_y < 0){
	// 			double a = target_theta - slam_roll;
	// 			vel.linear.x = V*dif*cos(a);
	// 			vel.linear.y = -V*dif*sin(a);
	// 		}
	// 		if(target_x < 0 && target_y < 0){
	// 			double a = target_theta + slam_roll;
	// 			vel.linear.x = V*dif*cos(a);
	// 			vel.linear.y = V*dif*sin(a);
	// 		}
	// 		if(dif < 0.05){
	// 			flag = 4;
	// 			//time1=ros::Time::now().toSec();
	// 		}
	// 	}
	// }
	// 	if(flag == 4){
	// 	vel.linear.y = 0;
	// 	vel.linear.x = 0;
	// 	//if(feed.angular.z==0)
	// 	{
	// 		if(vel.linear.x==0){
	// 			vel.linear.x=0;
	// 			fout_data_way_marker_x<<slam_x<<endl;
	// 			fout_data_way_marker_y<<slam_y<<endl;
	// 			fout_data_way_marker_z<<slam_z<<endl;
	// 			fout_data_way_marker_yaw<<slam_roll<<endl;
	// 			fout_data_way_kl<<kl_s<<endl;
	// 			flag = 5;
	// 		}
	// 	}
	// }
	// if(flag == 5){
	// 	//kl_pub.publish(scores);
	// 	ROS_INFO("end ~~~~~~~~~~~~~~~");
	// }
//}
	/////////////////pso////////////////////
	//cout<<"now_x: "<<now_x<<endl;
	//cout<<"now_y: "<<now_y<<endl;	
	// Point pbest_pos;
	// pbest_pos.x=now_x;
	// pbest_pos.y=now_y;
	// Point lbest_pos;
	// lbest_pos.x=0;
	// lbest_pos.y=0;
	//
	//ループ
	else if(flag==3){
		cout<<"wait-------"<<endl;
		kl_flag=1;
		scores_pub=scores;
		if(pso_flag==1){
			flag=4;
			time_begin=ros::Time::now().toSec();
			time2=ros::Time::now().toSec();
		}
		if(gbest>kl_s){
			gbest=kl_s;
			p.x=slam_x;
			p.y=slam_y;
		}
	}
	if(flag==5){
		cv_flag.num=3;
		ROS_INFO("end");
	}
	if(flag==4){
		cv_flag.num=2;
		time_now= ros::Time::now().toSec()-time_begin;
		double time_rec;
		time_rec = ros::Time::now().toSec()-time2;
		if(time_rec > 1)
		{
			time2=ros::Time::now().toSec();
			fout_data_way_marker_x<<slam_x<<endl;
			fout_data_way_marker_y<<slam_y<<endl;
			fout_data_way_marker_z<<slam_z<<endl;
			fout_data_way_marker_yaw<<slam_roll<<endl;
			fout_data_way_kl<<kl_s<<endl;
			num = num + 1;
			score.num = j;
			score.kl_score = kl_s;
			score.kl_x = slam_x;
			score.kl_y = slam_y;
			score.kl_z = slam_z;
			//score.kl_time = ros::Time::now().toSec()-time1;
			scores.KL.push_back(score);
		}
		cout<<"time_now: "<<time_now<<endl;
		cout<<"time_rec: "<<time_rec<<endl;
		cout<<"semi_score"<<semi_opt.kl_score<<endl;
        cout<<"semi_xxxx"<<semi_opt.kl_x<<endl;
        cout<<"semi_yyyy"<<semi_opt.kl_y<<endl;
		if(flag_gau==1){
			//kl_flag=1;
			scores_pub=scores;
			flag_gau=0;
		}
		//終了条件
		//if(sqrt(pso_score)<1){
		//	flag=2;
		//}

		if(time_now>50){//時間経過
			flag=5;
			ROS_INFO("time up~~~~~~~~~~~~~~~~~~~~");
		}
		else if(best_score < 1.0){
			flag=5;
			ROS_INFO("best position~~~~~~~~~~~~~~");
		}
		//if()ローカルベストがいまいち
		else{
			//粒子の位置の更新
			//new_x=now_x+new_vx;//simulation
			//new_y=now_y+new_vy;//simulation
			new_x=slam_x;
			new_y=slam_y;
			now_x=new_x;
			now_y=new_y;
			cout<<"new_x: "<<new_x<<endl;
			cout<<"new_y: "<<new_y<<endl;
			//粒子の速度の更新
			l.x=semi_opt.kl_x;
			l.y=semi_opt.kl_y;
			//l.x=//ガウス過程から付近の最小値
			//l.y=//
			//srand(time(NULL));//乱数
			double w0=0.5;
			double w1;//=0.5;//(double)rand()/RAND_MAX;
			double w2;//=0.3;//(double)rand()/RAND_MAX;
			double pi=3.141592;
			double k1=0.2;
			double k2=0.1;
			w1=k1*cos((pi*time_now)/(2*limit))*cos((pi*time_now)/(2*limit));
			w2=k2*sin((pi*time_now)/(2*limit))*sin((pi*time_now)/(2*limit));
			cout<<"w1: "<<w1<<endl;
			cout<<"w2: "<<w2<<endl;
			new_vx= 0.08*(w0*now_vx+w1*(l.x-new_x)+w2*(p.x-new_x));
			new_vy= 0.08*(w0*now_vy+w1*(l.y-new_y)+w2*(p.y-new_y));
			vel.linear.x=-new_vy;
			vel.linear.y=-new_vx;
			now_vx=new_vx;
			now_vy=new_vy;
			cout<<"new_vx: "<<new_vx<<endl;
			cout<<"new_vy: "<<new_vy<<endl;
			//pso_score=new_x*new_x+new_y*new_y;
			pso_score=kl_s;
			cout<<"score: "<<pso_score<<endl;
			if(gbest>pso_score){
				gbest=pso_score;
				//p.x=new_x;//simulation
				//p.y=new_y;//simulation
				p.x=slam_x;
				p.y=slam_y;
			}
			// cout<<"px: "<<p.x<<endl;
			// cout<<"py: "<<p.y<<endl;
			// cout<<"personal best: "<<gbest<<endl;
			cout<<"lx: "<<l.x<<endl;
			cout<<"ly: "<<l.y<<endl;
		}
		if(best_score > kl_s){
			best_score=kl_s;
		}
		if(gbest>kl_s){
			gbest=kl_s;
			p.x=slam_x;
			p.y=slam_y;
		}
	}*/
/////////////////pso////////////////////
////////////////////////////////////////////////////////////////////////////////
	//高さ制御
	double marker_disx;
	//marker_disx=slam_z-1;
	marker_disx=slam_z-way_z;
   	//marker_disx=odom_z-1;
	//if(flag!=5)
	{
		// if(marker_x==0){
		// 	vel.linear.z = 0;
		// 	ROS_INFO("NOT_FOUND");
		// }
		if(marker_disx>0.15){
			vel.linear.z = -(0.12*marker_disx*marker_disx+0.005);
		}
		else if(marker_disx<-0.15){
			vel.linear.z = (0.12*marker_disx*marker_disx+0.005);
		}
	ROS_INFO("flag: %d",flag );
	ROS_INFO("pso_flag: %d",pso_flag );
	ROS_INFO("pso_flag2: %d",pso_flag2 );
	ROS_INFO("new_flag: %d",new_flag );
	//ROS_INFO("gau_flag: %d",flag_gau );
	//ROS_INFO("num: %d",num );
	// cout<<"px: "<<p.x<<endl;
	// cout<<"py: "<<p.y<<endl;
	// cout<<"personal best: "<<gbest<<endl;
	// cout<<"personal best score: "<<best_score<<endl;
	ROS_INFO("kl_flag: %d",kl_flag );
	//  ROS_INFO("rflag: %d",rflag );
	//  ROS_INFO("tflag: %d",tflag );
	//  ROS_INFO("flag_semi: %d",flag_semi );
	// ROS_INFO("rrrrrrrrrrr: %f",r_suzu);
	// ROS_INFO("theta: %f",theta );
	 ROS_INFO("way_x: %f",way_x );
	 ROS_INFO("way_y: %f",way_y );
	 ROS_INFO("way_z: %f",way_z );
	//  ROS_INFO("slam_x: %f",slam_x );
	//  ROS_INFO("slam_y: %f",slam_y );
	//  ROS_INFO("target_x: %f",target_x );
	//  ROS_INFO("target_y: %f",target_y );
	// ROS_INFO("ab_y: %f",ab_y );
	// ROS_INFO("ab_z: %f",ab_z );
	 ROS_INFO("dif: %f",dif);
	//  ROS_INFO("target_theta: %f",target_theta);
	//  ROS_INFO("yaw  : %f",slam_roll);
	// ROS_INFO("marker_x: %f",marker_x );
	// ROS_INFO("vel.x: %f",vel.linear.x);
	// ROS_INFO("vel.y: %f",vel.linear.y);
	}
	//角速度　ビジュアルフィードバック
    vel.angular.z = feed.angular.z;
////////////////////////////////////////////////////////////////////////////////

	control->send(&vel);
	//std::cout<<"round :"<<round<<std::endl;
	break;
	}
    //fout_data_odom_z<<odom_z<<endl;
	 //ROS_INFO("SENT angular: %f", vel.angular.z);
	 //std::cout<<"Boxes.size"<<Boxes.size()<<std::endl;
	 //std::cout<<"round"<<round<<std::endl;
}


void Patroller::stop() {
	ROS_INFO("STOPPING PATROL");
	// flag = 0;
	// rflag = 0;
	// tflag = 0;
	patrolling = false;
	if( !control->isEnabled() ) control->toggle();
}

void Patroller::start(double altitude, double spacing) {
	ROS_INFO("STARTING PATROL");
	if( control->isEnabled() ) control->toggle();
	radius = (int) ceil(altitude);
	this->spacing = spacing;
	patrolling = true;

	current_state.distance = spacing;
	current_state.direction = CIRCLE;
	current_state.start = stats->getOdom()->pose.pose.position;
}
