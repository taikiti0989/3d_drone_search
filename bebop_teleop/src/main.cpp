#include "ManualControl.h"
#include "Patroller.h"
#include "StateTracker.h"
#include "Window.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include <kl_evaluation/angular.h>
//#include <aruco_mapping/aruco_mapping.h>
#include "aruco_mapping/ArucoMarker.h"
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <fstream>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
using namespace std;
using namespace cv;
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <kl_evaluation/kl_eval.h>
#include <bebop_test/flag.h>
#include <kl_evaluation/kl_suzu.h>
#include <kl_evaluation/kl_suzuki.h>
//#include <opencv2/opencv.hpp>
//Mat position_image(Size(600,600),CV_8UC3,Scalar(255,255,255));

string slam_position= "/home/das-note-1/catkin_ws/src/bebop_data/slam_pose.csv"; //20200924 yokomatsu
ofstream slam_position_csv; //20200924 yokomatsu

string file_data_time= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_time.csv";
ofstream fout_data_time;
string file_data_way_time= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_way_time.csv";
ofstream fout_data_way_time;
string file_data_marker_x= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_slam_x.csv";
ofstream fout_data_marker_x;
string file_data_marker_y= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_slam_y.csv";
ofstream fout_data_marker_y;
string file_data_marker_z= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_slam_z.csv";
ofstream fout_data_marker_z;
string file_data_marker_yaw= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_slam_yaw.csv";
ofstream fout_data_marker_yaw;
string file_data_kl= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_kl.csv";
ofstream fout_data_kl;
string file_data_way_marker_x= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_way_marker_x.csv";
ofstream fout_data_way_marker_x;
string file_data_way_marker_y= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_way_marker_y.csv";
ofstream fout_data_way_marker_y;
string file_data_way_marker_z= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_way_marker_z.csv";
ofstream fout_data_way_marker_z;
string file_data_way_marker_yaw= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_way_marker_yaw.csv";
ofstream fout_data_way_marker_yaw;
string file_data_way_kl= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_way_kl.csv";
ofstream fout_data_way_kl;
string file_data_map_x= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_map_x.csv";
ofstream fout_data_map_x;
string file_data_map_y= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_map_y.csv";
ofstream fout_data_map_y;
string file_data_map_z= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_map_z.csv";
ofstream fout_data_map_z;
string file_data_map_kl= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_map_kl.csv";
ofstream fout_data_map_kl;

string file_data_target_x= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_target_x.csv";
ofstream fout_data_target_x;
string file_data_target_y= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_target_y.csv";
ofstream fout_data_target_y;
string file_data_target_z= "/home/das-note-1/catkin_ws/src/bebop_data/bebop_target_z.csv";
ofstream fout_data_target_z;
geometry_msgs::Twist feed;
geometry_msgs::Twist feed_red;
//aruco_mapping::ArucoMarker bebop_positon;
double odom_x;
double odom_y;
double odom_z;
double odom_yaw;
double marker_x;
double marker_y;
double marker_z;
double marker_yaw;
double marker_yaw2;
double marker_pitch;
double pai=3.141592;
double kl_s;
double slam_x;
double slam_y;
double slam_z;
double slam_yaw;
double slam_roll;
kl_evaluation::kl_suzu score;
kl_evaluation::kl_suzuki scores;
kl_evaluation::kl_suzuki scores_pub;
kl_evaluation::kl_suzu semi_opt;
bool _isTakeOff=false;
bool _isTracking=false;
extern int flag;
extern int kl_flag;
int flag_semi=0;
extern int pso_flag;
int flag_gau=0;
double klkl;
bebop_test::flag cv_flag;
// Point position;
const bool trackOff = false;
const bool trackOn = true;
const std::string tracking = "/bebop/tracking";

/////odometry callback関数////////////
void msgCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
//データの格納
odom_x=msg->pose.pose.position.x;
odom_y=msg->pose.pose.position.y;
odom_z=msg->pose.pose.position.z;
//yaw角変換
  ////////////////////transfer quanternion to Euler angle//////////////////
  double qx1,qy1,qz1,qw1;
  qx1=msg->pose.pose.orientation.x;
  qy1=msg->pose.pose.orientation.y;
  qz1=msg->pose.pose.orientation.z;
  qw1=msg->pose.pose.orientation.w;
  double siny_cosp = +2.0 * (qw1 * qz1 + qx1 * qy1);
  double cosy_cosp = +1.0 - 2.0 * (qy1 * qy1 + qz1 * qz1);
  odom_yaw=atan2(siny_cosp, cosy_cosp);
}
void angularCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    feed.angular.z=msg->angular.z;
    //ROS_INFO("kl_angularcallback");
}
void angularCallback2(const geometry_msgs::Twist::ConstPtr &msg)
{
    feed_red.angular.z=msg->angular.z;
    //ROS_INFO("red_angularCallback");
}
void klCallback(const kl_evaluation::kl_eval::ConstPtr &klmsg)
{
    klkl=klmsg->kl;
    if(klkl>0){
        kl_s=klmsg->kl;
        //ROS_INFO("klklklklklklklklklkl");
    }
}
void markerCallback(const aruco_mapping::ArucoMarker::ConstPtr &msg)
{
        //bebop_positon.global_camera_pose.position.z=msg->global_camera_pose.position.z;
        marker_x = msg->global_camera_pose.position.x;
        marker_y = msg->global_camera_pose.position.y;
        marker_z = msg->global_camera_pose.position.z;
        // ROS_INFO("marker_x: %f",marker_x );
        // ROS_INFO("marker_y: %f",marker_y );
        // ROS_INFO("marker_z: %f",marker_z );
        double q0,q1,q2,q3;
        q0=msg->global_camera_pose.orientation.x;//q0
        q1=msg->global_camera_pose.orientation.y;//q1
        q2=msg->global_camera_pose.orientation.z;//q2
        q3=msg->global_camera_pose.orientation.w;//q3
        double siny_cosp = +2.0 * (q3 * q0 + q2 * q1);
        double cosy_cosp = +1.0 - 2.0 * (q3 * q3 + q2 * q2);
        marker_yaw=atan2(siny_cosp, cosy_cosp);
        double yaw = marker_yaw*180/pai;
        double marker_roll = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
        double roll = marker_roll*180/pai;
        marker_pitch = asin(2*(q0*q2-q3*q1));
        double pitch = marker_pitch*180/pai;
       // marker_yaw2 = 1.570796 - marker_yaw;
        // fout_data_marker_x<<marker_x<<endl;
        // fout_data_marker_y<<marker_y<<endl;
        // fout_data_marker_z<<marker_z<<endl;
        // fout_data_marker_yaw<<marker_pitch<<endl;
        // ROS_INFO("marker_roll rad  : %f",marker_roll );
        // ROS_INFO("marker_roll deg  : %f",roll );
        // ROS_INFO("marker_pitch rad  : %f",marker_pitch );
        // ROS_INFO("marker_pitch deg  : %f",pitch );
        // ROS_INFO("marker_yaw rad  : %f",marker_yaw );
        // ROS_INFO("marker_yaw deg  : %f",yaw );
}
void semi_klCallback(const kl_evaluation::kl_suzu::ConstPtr &msg)
{
        double ls = msg->kl_score;
        if(ls > 0)
        {
                semi_opt.kl_score = msg->kl_score;
                semi_opt.kl_x = msg->kl_x;
                semi_opt.kl_y = msg->kl_y;
                semi_opt.kl_z = msg->kl_z;
                pso_flag=1;
                flag_gau=1;
                // cout<<"score"<<semi_opt.kl_score<<endl;
                // cout<<"xxxx"<<semi_opt.kl_x<<endl;
                // cout<<"yyyy"<<semi_opt.kl_y<<endl;
                // cout<<"zzzz"<<semi_opt.kl_z<<endl;
                // cout<<"llllllllllllll"<<endl;
                // flag_semi = 1;
        }
}
void orb_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
        slam_x = msg->pose.position.x;
        slam_y = msg->pose.position.y;
        slam_z = msg->pose.position.z;
        slam_position_csv << slam_x <<","<< slam_y <<","<< slam_z <<endl;
        ROS_WARN("slam pose output");
        
        //position.x = slam_x;
        //position.y = slam_y;
        double q0,q1,q2,q3;
        q0=msg->pose.orientation.x;//q0
        q1=msg->pose.orientation.y;//q1
        q2=msg->pose.orientation.z;//q2
        q3=msg->pose.orientation.w;//q3
        double siny_cosp = +2.0 * (q3 * q0 + q2 * q1);
        double cosy_cosp = +1.0 - 2.0 * (q3 * q3 + q2 * q2);
        slam_yaw=atan2(siny_cosp, cosy_cosp);
        //cout<<"yaw: "<<slam_yaw<<endl;
        //double yaw = marker_yaw*180/pai;
        slam_roll = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
        double roll = slam_roll*180/pai;
        //cout<<"roll: "<<slam_roll<<endl;
        marker_pitch = asin(2*(q0*q2-q3*q1));
        double pitch = marker_pitch*180/pai;
        // circle(position_image,position,5,Scalar(0,0,0));
        // cv::imshow("map",position_image);
        // cv::waitKey(100);
        //cout<<"pitch: "<<marker_pitch<<endl;
        // fout_data_marker_x<<slam_x<<endl;
        // fout_data_marker_y<<slam_y<<endl;
        // fout_data_marker_z<<slam_z<<endl;
        // fout_data_marker_yaw<<slam_roll<<endl;
        //ROS_INFO("22222222");
}
//////ファイル宣言
int main(int argc, char** argv) {
        ros::init(argc, argv, "bebop_odom");
        slam_position_csv.open(slam_position.c_str()); //20200924 yokomatsu

        fout_data_marker_x.open(file_data_marker_x.c_str());
        fout_data_marker_y.open(file_data_marker_y.c_str());
        fout_data_marker_z.open(file_data_marker_z.c_str());

        fout_data_map_x.open(file_data_map_x.c_str());
        fout_data_map_y.open(file_data_map_y.c_str());
        fout_data_map_z.open(file_data_map_z.c_str());
        fout_data_map_kl.open(file_data_map_kl.c_str());
        fout_data_target_x.open(file_data_target_x.c_str());
        fout_data_target_y.open(file_data_target_y.c_str());
        fout_data_target_z.open(file_data_target_z.c_str());
        fout_data_marker_yaw.open(file_data_marker_yaw.c_str());
        fout_data_way_marker_x.open(file_data_way_marker_x.c_str());
        fout_data_way_marker_y.open(file_data_way_marker_y.c_str());
        fout_data_way_marker_z.open(file_data_way_marker_z.c_str());
        fout_data_way_marker_yaw.open(file_data_way_marker_yaw.c_str());
        fout_data_way_kl.open(file_data_way_kl.c_str());
        fout_data_kl.open(file_data_kl.c_str());
        fout_data_way_time.open(file_data_way_time.c_str());
        fout_data_time.open(file_data_time.c_str());
        ros::NodeHandle nh;
        ros::NodeHandle local_nh("~");    
        local_nh.param( "font_path", Window::font_path, std::string("/usr/share/fonts/truetype/freefont/FreeSans.ttf") );
        local_nh.param( "circle_path", Window::circle_path, std::string("/home/das-note-1/catkin_ws/src/bebop_teleop/circle.bmp") );

        bool fail = false;
        input = new Input();
        stats = new StateTracker();
        window = new Window(fail);
        control = new ManualControl();
        patroller = new Patroller();

        control->advertise(nh);

        image_transport::ImageTransport it(nh);
        image_transport::TransportHints hints("compressed", ros::TransportHints(), local_nh);
        image_transport::Subscriber sub = it.subscribe("bebop/image_raw", 1, &Window::updateVideoTexture, window, hints);
        ros::Publisher kl_pub2 = nh.advertise<kl_evaluation::kl_suzuki>("kl_suzuki_msg",10);
        ros::Publisher flag_pub = nh.advertise<bebop_test::flag>("/flag_msg",10);
        ros::Subscriber ros_tutorial_sub = nh.subscribe("bebop/odom", 10, msgCallback);
//ros::Subscriber ros_tutorial_sub2 = nh.subscribe("/aruco_markers", 10, msgCallback2);
//void angularCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxMessage)
        ros::Subscriber ros_tutorial_sub2 = nh.subscribe( "angular_msg", 1, angularCallback);
        ros::Subscriber ros_tutorial_sub3 = nh.subscribe( "/hsv/angular_msg", 1, angularCallback2);
        ros::Subscriber ros_tutorial_sub4 = nh.subscribe( "aruco_poses", 1, markerCallback);
        ros::Subscriber ros_tutorial_sub5 = nh.subscribe( "kl_msg", 1, klCallback);
        ros::Subscriber ros_tutorial_sub6 = nh.subscribe( "semi_opt", 1, semi_klCallback);
        ros::Subscriber ros_tutorial_sub7 = nh.subscribe( "/bebop/true_pose", 1, orb_Callback);

        stats->subscribe(nh);

        ros::Rate r(30);


        // fprintf(stdout, "\nKeys:\nW: forward\tS: backward\nA: left\t\tD: right\nSPACE: up\tLSHIFT: down\nCTRL: land\tRSHIFT: takeoff\nUP: camera up\tDOWN: camera down\nLEFT: rot left\tRIGHT: rot right\nENTER: emergency rotor shutdown\n2: start video\t3: end video\n1: Take a camera snapshot\nUse I, J, K, and L sparingly for arial flips. You can also use '[' and ']' to start and stop autohome navigation.\nEnsure SDL Window is focused for input to be processed!\n");
        while( ros::ok() && window->ok() ) {
                ros::spinOnce();
                eventPoll();
                control->publishVel();
                patroller->patrol();
                control->publishCam();
                window->update();
		nh.setParam(tracking,_isTracking);
                //cout<<"flag"<<flag<<endl;
                flag_pub.publish(cv_flag);
                if(kl_flag==1){
                // score.kl_score = 2.44;
		// score.kl_x = 1;
		// score.kl_y = -1.18;
		// score.kl_z = 1.57;
		// scores.KL.push_back(score);
                        //scores_pub=scores;
                        kl_pub2.publish(scores);
                        //kl_flag=0;
                        cout<<"-----------------------"<<endl;
                        cout<<"sizeeeeeeeeeeeee"<<scores.KL.size()<<endl;
                        // if(semi_opt.kl_score != 0){
                        //         flag = 4;
                        // }
                }
                cout<<"sizeeeeeeeeeeeee"<<scores.KL.size()<<endl;
                // ROS_INFO( "STATS: Batt: %d%% Wifi: %d GPS: %s\nGPS: (Latitude: %0.6f Longitude: %0.6f)\nVELX: %0.3f VELY: %0.3f VELZ: %0.3f", stats->getBattery(), stats->getWifiStrength(), stats->hasFix() ? "Has Fix" : "No Fix", stats->getLatitude(), stats->getLongitude(), stats->getXVelocity(), stats->getYVelocity(), stats->getZVelocity() );
                r.sleep();
        }


        ros::shutdown();

        delete window;
        delete patroller;
        delete control;
        delete stats;
        delete input;

        slam_position_csv.close(); //20200924 yokomatsu

        fout_data_marker_x.close();
        fout_data_marker_y.close();
        fout_data_marker_z.close();
        fout_data_map_x.close();
        fout_data_map_y.close();
        fout_data_map_z.close();
        fout_data_map_kl.close();
        fout_data_target_x.close();
        fout_data_target_y.close();
        fout_data_target_z.close();
        fout_data_marker_yaw.close();
        fout_data_way_marker_x.close();
        fout_data_way_marker_y.close();
        fout_data_way_marker_z.close();
        fout_data_way_marker_yaw.close();
        fout_data_way_kl.close();
        fout_data_kl.close();
        fout_data_way_time.close();
        fout_data_time.close();  
        return 0;
}
