#ifndef __PATROL_H__
#define __PATROL_H__

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#define FORWARD 0
#define RIGHT 1
#define BACKWARD 2
#define LEFT 3
///by lan
#define CIRCLE 4
//by suzuki
#define TRACK 5

struct State {
	double distance;
	short direction;
	geometry_msgs::Point start;
};

class Patroller {
public:
	Patroller(void);
	~Patroller(void);
	void destroy(void);

	void patrol(void);
	void start(double, double);
	void stop(void);

private:
	State current_state;
	int radius = 0;
	double spacing = 0;
	double speed = 0;
	bool patrolling = false;
        double current_time;
	int flag=0;
	double start_time;

	void checkState(void);
	void nextState(void);
};

extern Patroller* patroller;

#endif
