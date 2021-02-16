#include <ros/ros.h>
#include <cv.h>
#include <math.h>
#include <iostream>
#include <sstream>
////////////////keyboard
#include <kl_evaluation/Key.h>
#include <kl_evaluation/Face.h>
#include <kl_evaluation/FaceArray.h>
#include <kl_evaluation/BoundingBox.h>
#include <kl_evaluation/BoundingBoxes.h>
#include <kl_evaluation/position.h>
#include <kl_evaluation/positions.h>
#include <kl_evaluation/angular.h>
#include <kl_evaluation/kl_eval.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <opencv2/objdetect/objdetect.hpp>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <kl_evaluation/vbem.h>
#include <boost/math/special_functions/digamma.hpp>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <kl_evaluation/functions.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "find_color/box_suzuki.h"
#include "find_color/boxes.h"
using namespace std;
using namespace cv;
int fps_lan=4;
/////////////////////////by takeuchi//////////////////////////////////
string file_data_x= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_pose_x.csv";
ofstream fout_data_x;
string file_data_y= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_pose_y.csv";
ofstream fout_data_y;
string file_data_z= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_pose_z.csv";
ofstream fout_data_z;
string file_data_yaw= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_pose_yaw.csv";
ofstream fout_data_yaw;
string file_data_differ_x= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_differ_x.csv";
ofstream fout_data_differ_x;
string file_data_differ_y= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_differ_y.csv";
ofstream fout_data_differ_y;
string file_data_bboxheight= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_bboxheight.csv";
ofstream fout_data_bboxheight;
string file_data_time= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_time.csv";
ofstream fout_data_time;
string file_data_score= "/home/das-note-1/catkin_ws/src/bebop_data/kl_bebop_score.csv";
ofstream fout_data_score;
double ros_begin;
double ros_duration;
String record_path = "/home/das-note-5/kl_data_save/";

///with score
cv::VideoWriter writer1(record_path + "image_lan.avi",CV_FOURCC('X','V','I','D'),fps_lan,Size(120,80));//640
///composition map
cv::VideoWriter writer2(record_path + "composition_map.avi",CV_FOURCC('X','V','I','D'),fps_lan,Size(300,300));
///origin
cv::VideoWriter writer3(record_path + "image_origin.avi",CV_FOURCC('X','V','I','D'),fps_lan,Size(400,480));//640

int movement=0;
vector<Body_lan> Bodies_lan;
//vector<Body_lan> Bodies_lan2;
int flag_person=0;
Mat cloudImage;
int image_row=480;
int image_col=856;//856
const int frameMiddleX=428;
const int frameMiddleY=240;
//////////////////////////////////by takeuchi///////////////////////////////////
double direction_yaw;
double px,py,pz;
double k;
double kk;
double ls=99.0;
int cishu=0;
int cishuf=0;
vector<vector<double> > pingjunf;
vector<double> q;
vector<double> yilunf;
Point my_position=Point(600,900);
Point2f Goalr;
Point2f Goal_point=Point(600,900);
Point2f Goal_point2=Point(600,900);
vector<Object> preface_body;
vector<Object> preface;
int stop_flag=1;
double angvellan=0;
double direlan=0;
double speedlankai=0;
double present_score;
Point pixcenter=Point(428,240);//(428,240)
vector<Object> face_order;
vector<Person> person;
float xdif;
find_color::box_suzuki BOX;
//////////////////////////////////////////////////
typedef vector <double> Vec1;
typedef vector <Vec1> Mtx;
#define SZ(a) ((int)((a).size()))

Vec1 alphae;
Vec1 betae;
Vec1 nyue;
Mtx  me;
vector<Mtx> We;
int frame_counter=0;
vector<int> groupnumber;
vector<Scalar> facecolor;
vector<int> whichem;
Mtx dataFrame;
vector<Object> cluster_body;
Mat cameraImg;
vector<darknet_ros_msgs::BoundingBox> Boxes;//vector宣言
double sigmoid(double gain, double x) {
    return 1.0 / (1.0 + exp(-gain * x ));
}
class depth_estimater{

public:
    depth_estimater();
    ~depth_estimater();
   
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void box_receiver_cb(kl_evaluation::BoundingBoxes box_array);
   // void imageCallback (const sensor_msgs::PointCloud2ConstPtr& input); 
    void keyboard_lan_subscriber_callback(kl_evaluation::Key key_lan);
    void msgCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxMessage);
	void centerCallback(const find_color::box_suzuki::ConstPtr &centerMessage);
	void pso_suzuki();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, target_sub,keyboard_sub,ros_odom_sub,box_sub,center_sub;   // cloud_depth_sub,
    ros::Publisher chatter_pub,angular_pub,kl_pub;
	geometry_msgs::Twist last;
    
};

depth_estimater::depth_estimater(){
    target_sub = nh.subscribe<kl_evaluation::BoundingBoxes>("/darknet_ros/bounding_boxes",1,&depth_estimater::box_receiver_cb,this);
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/bebop/image_raw", 1, &depth_estimater::rgbImageCallback, this);  //  camera/color,bebop
    //cloud_depth_sub = nh.subscribe("/camera/depth_registered/points", 1, &depth_estimater::imageCallback,this);
    chatter_pub = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);//cmd_vel
	angular_pub = nh.advertise<geometry_msgs::Twist>("angular_msg", 1);
	kl_pub = nh.advertise<kl_evaluation::kl_eval>("kl_msg", 1);
    keyboard_sub = nh.subscribe <kl_evaluation::Key>("/keyboard/keydown",10,&depth_estimater::keyboard_lan_subscriber_callback,this);
	//odom
	ros_odom_sub = nh.subscribe("/bebop/odom", 10, &depth_estimater::msgCallback,this);
	box_sub = nh.subscribe( "darknet_ros/bounding_boxes", 1, &depth_estimater::boxCallback,this);
	center_sub = nh.subscribe( "/box_size", 1, &depth_estimater::centerCallback,this);
}
/*void depth_estimater::send(geometry_msgs::Twist* msg) {
	ROS_INFO("PUBLISH MSG");
	last = *msg;
	chatter_pub.publish(*msg);
}*/
 
depth_estimater::~depth_estimater(){
}
void depth_estimater::centerCallback(const find_color::box_suzuki::ConstPtr &centerMessage){
	BOX.center_x=centerMessage->center_x;
	
	//cout<<"box centerx:"<<<<endl;
	
}
void depth_estimater::boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxMessage)
{
    darknet_ros_msgs::BoundingBox boxMsg;//boxMsg宣言  
    Boxes.clear();
    for(int i=0;i<boxMessage->bounding_boxes.size();i++)//検出した数まで繰り返す
    {        
        if(boxMessage->bounding_boxes[i].Class=="person")//traffic light person
        //if(boxMessage->bounding_boxes[i].probability>=0.10)//simulation :person,traffic light,dining table,
        {
        boxMsg=boxMessage->bounding_boxes[i];
        Boxes.push_back(boxMsg);//BoxesにboxMsg情報を格納
        }
    }
	//ROS_INFO("444444444444444444");
}

////////// accept the keyboard input for an emergency stop////////////////////////
void depth_estimater::keyboard_lan_subscriber_callback(kl_evaluation::Key key_lan)
{
  movement=key_lan.code;
  //cout<<"key:"<<movement<<endl;
}

void depth_estimater::msgCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("mecanum.position.x = %f", msg->pose.pose.position.x);   // Prints the 'x coordinate' in terminal
  //ROS_INFO("mecanum.position.y = %f", msg->pose.pose.position.y);  // Prints the 'y coordinate' in terminal
  //ROS_INFO("mecanum.orientation.w = %f", msg->pose.pose.orientation.w); // Prints the 'w coordinate' in terminal

  fout_data_x<<msg->pose.pose.position.x<<endl;  //record the x coordinate in csv file
  fout_data_y<<msg->pose.pose.position.y<<endl;  //record the y coordinate in csv file
  fout_data_z<<msg->pose.pose.position.z<<endl;  //record the z coordinate in csv file
  //fout_data_x<<msg->pose.pose.position.x<<endl;  //record the x coordinate in csv file
  //fout_data_y<<msg->pose.pose.position.y<<endl;  //record the y coordinate in csv file
 

  ////////////////////transfer quanternion to Euler angle//////////////////

  //double qx,qy,qz,qw;
  //qx=msg->pose.pose.orientation.x;
  //qy=msg->pose.pose.orientation.y;
  //qz=msg->pose.pose.orientation.z;
  //qw=msg->pose.pose.orientation.w;
  //double siny_cosp = +2.0 * (qw * qz + qx * qy);
  //double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);  
  //direction_yaw=atan2(siny_cosp, cosy_cosp);

  //px=msg->pose.pose.position.x;
  //py=msg->pose.pose.position.y;
  //pz=msg->pose.pose.position.z;

  //fout_data_w<<direction_yaw<<endl;
  double qx1,qy1,qz1,qw1;
  qx1=msg->pose.pose.orientation.x;
  qy1=msg->pose.pose.orientation.y;
  qz1=msg->pose.pose.orientation.z;
  qw1=msg->pose.pose.orientation.w;
  double siny_cosp = +2.0 * (qw1 * qz1 + qx1 * qy1);
  double cosy_cosp = +1.0 - 2.0 * (qy1 * qy1 + qz1 * qz1);
  direction_yaw=atan2(siny_cosp, cosy_cosp);
  //データの格納
  fout_data_yaw<<direction_yaw<<endl;  //record the z coordinate in csv file
}

///////////record the pixel coordinates of detected "humans" by yolo/////////////
void depth_estimater::box_receiver_cb(kl_evaluation::BoundingBoxes box_array)
{
	Bodies_lan.clear();
	int nl=box_array.bounding_boxes.size();
	for(int i=0;i<nl;i++)
	{
		if(box_array.bounding_boxes[i].Class=="person")//bottle person  traffic light
		{
			Body_lan body_lan;
			body_lan.xmin=box_array.bounding_boxes[i].xmin;
			body_lan.ymin=box_array.bounding_boxes[i].ymin;
			body_lan.xmax=box_array.bounding_boxes[i].xmax;
			body_lan.ymax=box_array.bounding_boxes[i].ymax;
			body_lan.Class=box_array.bounding_boxes[i].Class;
			// cout<<"body_lan.xmin :"<<body_lan.xmin<<endl;
			// cout<<"body_lan.ymin :"<<body_lan.ymin<<endl;
			// cout<<"body_lan.xmax :"<<body_lan.xmax<<endl;
			// cout<<"body_lan.ymax :"<<body_lan.ymax<<endl;
			// cout<<"body_lan.class :"<<body_lan.Class<<endl;
			Bodies_lan.push_back(body_lan);
			flag_person=1;
		}
		else 
			flag_person=0;
		
	}
		if(Bodies_lan.size()>1){
			float cen_x=0;
			float sum_x=0;
			for(int i=0;i<Bodies_lan.size();i++){
				cen_x=(Bodies_lan[i].xmin+Bodies_lan[i].xmax)/2.0;
				sum_x=sum_x+cen_x;
				//cout<<"cen_x :"<<cen_x<<endl;
			}
			float Cen_x;
			Cen_x=sum_x/Bodies_lan.size();
			//cout<<"sum_x :"<<sum_x<<endl;
			xdif=frameMiddleX-Cen_x;
			cout<<"difx :"<<xdif<<endl;
		}
	//cout<<"Bodies_lan0.size()="<<Bodies_lan.size()<<endl;
}
void depth_estimater::pso_suzuki(){

}

void depth_estimater::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);vector<Object> target_vec;
		target_vec.clear();
		vector<double> testper;
		testper.clear();
		//vector<Person> person;
		person.clear();
		//vector<Object> face_order;
		face_order.clear();
		vector<size_t> order_num;///////////////for human order
		order_num.clear();

		//Mat cameraImg;
		Mat resoImg;
		cameraImg = cv_ptr->image;
		resoImg=cameraImg.clone();
		writer3<<resoImg;
		imwrite("/home/das-note-1/catkin_ws/photo.jpg",resoImg);
		//imshow("rgb",cameraImg);
		//waitKey(3);
		//cout<<"here"<<endl;
		///////calculate information of boundingboxes of detected humans/////////////
		if(Bodies_lan.size()!=0)
		{
			for(int i=0;i<Bodies_lan.size();i++)
			{   
				Object target_rec;
				cv::rectangle(cameraImg, Point(Bodies_lan[i].xmin,Bodies_lan[i].ymin), Point(Bodies_lan[i].xmax,Bodies_lan[i].ymax), CV_RGB(0, 255, 0), 2);
				target_rec.pixPoint.x=((Bodies_lan[i].xmin+Bodies_lan[i].xmax)/2)+xdif;//中心///////////////////
				target_rec.pixPoint.y=(Bodies_lan[i].ymin+Bodies_lan[i].ymax)/2;//中心
				cv::circle(cameraImg,Point(target_rec.pixPoint.x,target_rec.pixPoint.y),3,Scalar(255,100,0),-1,8,0);

				target_rec.boundingbox.x=Bodies_lan[i].xmin+xdif;/////////////////
				target_rec.boundingbox.y=Bodies_lan[i].ymin;
				target_rec.boundingbox.width=Bodies_lan[i].xmax-Bodies_lan[i].xmin;
				target_rec.boundingbox.height=Bodies_lan[i].ymax-Bodies_lan[i].ymin;

				target_rec.face_width=target_rec.boundingbox.width;
				target_rec.face_height=target_rec.boundingbox.height;
				target_vec.push_back(target_rec);
				testper.push_back(target_rec.pixPoint.x);
			}
		

			/////////////////////////////////////////////////////////////////////
			/////////////////////////re-order the targets/////////////////////////
			/////////////It is not a must to re-order the targets here////////////
			getSortOrder(testper,order_num,true);
			for(int i=0;i<target_vec.size();i++)
			{
				face_order.push_back(target_vec[order_num[i]]);
			}

			for(int i=0;i<target_vec.size();i++)
			{
				Person people;
				people.face_pixPoint=target_vec[i].pixPoint;
				people.face_width=target_vec[i].boundingbox.width;
				people.face_height=target_vec[i].boundingbox.height;
				people.rectangle=target_vec[i].boundingbox;
				person.push_back(people);
			}
			int center_sum_x=0;
			int center_sum_y=0;
			if(person.size()>0)
			{
				for(size_t i;i<person.size();i++)
				{
					center_sum_x=person[i].face_pixPoint.x+center_sum_x;
					center_sum_y=person[i].face_pixPoint.y+center_sum_y;
				}

				pixcenter.x=center_sum_x/person.size();
				pixcenter.y=center_sum_y/person.size();

			}
			//cout<<"person size 0="<<person.size()<<endl;
			//ROS_INFO("here!!!");

			////////////composition evaluation for the present scene////////////////////////
			//////////////////result will be display on the rgb image///////////////////////
			///////////// if target number is smaller than 4, do not execute clustering ///////
			if(movement==49)
			{
				//ROS_INFO("1111111111111111111");
				if(Boxes.size()!=0){
					//ros_duration = ros::Time::now().toSec()-ros_begin;
					long int center_x_sum=0;
        			long int center_y_sum=0;
        			long int groupcenter_x=0;
    				long int groupcenter_y=0;
        			for(int i=0;i<Boxes.size();i++){
            			point center;
            			center.x=(Boxes[i].xmin+Boxes[i].xmax)/2.0;
            			center.y=(Boxes[i].ymin+Boxes[i].ymax)/2.0;
            			center_x_sum=center_x_sum+center.x;
            			center_y_sum=center_y_sum+center.y;
					}
        			groupcenter_x=center_x_sum/Boxes.size();
        			groupcenter_y=center_y_sum/Boxes.size();
        			std::vector<double> group_ymin;
        			std::vector<size_t> ymin_size;
        			std::vector<double> group_ymax;
        			std::vector<size_t> ymax_size;
        			group_ymin.clear();
        			group_ymax.clear();
        			ymin_size.clear();
        			ymax_size.clear();
        			for(size_t i=0;i<Boxes.size();i++){
            			group_ymin.push_back(Boxes[i].ymin);
            			group_ymax.push_back(Boxes[i].ymax);
					}
        			getSortOrder(group_ymin,ymin_size,true);
        			getSortOrder(group_ymax,ymax_size,true);
					////////////////////center計算///////////////////////
        			long int xMiddle = groupcenter_x;
        			long int yMiddle = groupcenter_y;
        			long int xDifference = xMiddle - frameMiddleX;
        			long int yDifference = yMiddle - frameMiddleY;
        			long int xDifferenceAbsolute = labs(xMiddle - frameMiddleX);
        			long int yDifferenceAbsolute = labs(yMiddle - frameMiddleY);
        			long int bboxHeight = group_ymax[group_ymax.size()-1] - group_ymin[0];
        			//ROS_INFO("bboxHeight : %ld", bboxHeight);
        			ROS_INFO("xdiffernce : %ld", xDifference);
					ROS_INFO("groupcenter_x : %ld", groupcenter_x);
        			//ROS_INFO("ydiffernce : %ld", yDifference);
					/////////////////////saving data////////////////////////////////////
        			fout_data_differ_x<<xDifference<<endl;//record the differ_x coordinate in csv file
        			// fout_data_differ_y<<yDifference<<endl;//record the differ_y coordinate in csv file
        			// fout_data_bboxheight<<bboxHeight<<endl;//record the differ_y coordinate in csv file
					ros_duration = ros::Time::now().toSec()-ros_begin;
					//ROS_INFO("Time : %f", ros_duration);
					fout_data_time<<ros_duration<<endl;//record the time
					////////////////width////////////////////
    				/*if(xDifferenceAbsolute > 20){
            			if(xDifference > 0 ){
                			last.angular.z = 0.48*sigmoid(0.03,xDifferenceAbsolute,120)+0.02;
                			ROS_INFO("CCW");}
            			else if(xDifference < 0){
                			last.angular.z = -(0.48*sigmoid(0.03,xDifferenceAbsolute,120)+0.02);
                			ROS_INFO("CW");}
					}*/
					if(xDifferenceAbsolute > 20){
                		last.angular.z = 0.8*sigmoid(0.006,xDifference)-0.4;
                		// ROS_INFO("angular");
						}
         			else{
            			last.angular.z = 0;
            			//ROS_INFO("WIDTH END");
         				}
				}
////////////////////////////////////////////////////
				calKLD_multi(target_vec,ls);

				////////////////////////////record evalution result to the rgb image///////////////// 
				ostringstream information1;
				information1.precision(3);
				information1<<ls;
				cv::putText(cameraImg, "KLD_C: " + information1.str(), cv::Point(25,25), CV_FONT_NORMAL, 0.75, Scalar(255,50,50),1,1);
				fout_data_score<<ls<<endl;//record the kl_score coordinate in csv file
				kl_evaluation::kl_eval eva;
				eva.kl = ls;
				kl_pub.publish(eva);
				// cout<<"kl"<<ls<<endl;
				///////////////////////save the images as a avi video////////////////////////////////
				writer1<<cameraImg;
				///////////////////////show the rgb image ///////////////////////////
Mat display;
//Mat display(Size(360,240),CV_8UC3,Scalar(255,255,255));
resize(cameraImg,display,Size(),0.8,0.8);
				imshow("result",display);

				waitKey(3);
				angular_pub.publish(last);
			}

dataFrame.clear();
if(Bodies_lan.size()>1&&stop_flag==1)
	{
	    for(size_t l=0;l<Bodies_lan.size();l++)
    	{
		
			Point2f points0;
			points0.x=Bodies_lan[l].center_x*1.0;
			points0.y=Bodies_lan[l].center_y*1.0;
			Vec1 tyu;
			tyu.clear();
			tyu.push_back(points0.x);
			tyu.push_back(points0.y);
			dataFrame.push_back(tyu);

	    }

		//cout<<"size="<<Bodies_lan.size()<<endl;
		if(movement==50&&Bodies_lan.size()>3)
		{
			/* code */
			/////////////////////////////////////////////////////////////////////
			////////////////////////////VB-EM Algorithm//////////////////////////
			/////////////////////////////////////////////////////////////////////
			
			vector<Point> centerss_lan;
			centerss_lan.clear();
			vector<Point> sizess_lan;
			sizess_lan.clear();
			vector<int> angless_lan;
			angless_lan;
			frame_counter++;
			int groups_em;
			groupnumber.clear();
			facecolor.clear();

			VBEM_GMM *em=new VBEM_GMM();
			//////////////////class number and repeat count//////////////////////
			const int numLoops=50;
			const int numClasses=4;
		
			em->initialize2(dataFrame,numClasses,frame_counter);

			/////////////////////////////execute/////////////////////////////////
			int counter_em=0;
			em->run(numLoops,counter_em);

			/////////////////////////////output//////////////////////////////////
			Mtx output_em;
			output_em.clear();
			em->printGammaNK(groups_em,whichem,centerss_lan,sizess_lan,angless_lan);
			em->outputGamma(output_em);
			//cout<<"groups="<<groups_em<<endl;

			/////////////////////////drawing clustering results//////////////////	

			int rr,gg,bb;
			double psxx=0;
			double psyy=0;
		
			/////////////////////////calculate targets center////////////////////
			for(size_t i=0;i<face_order.size();i++)
			{
				psxx+=face_order[i].pixPoint.x;
				psyy+=face_order[i].pixPoint.y;
			}
			double psize;
			psize=(double)(face_order.size()*1.0);
			psxx=psxx/psize;
			psyy=psyy/psize;

			for(size_t i=0;i<face_order.size();i++)
			{
				if(groups_em>1&&groups_em<numClasses)
				{
					vector<double> line_lan;
					line_lan.clear();
					vector<size_t> order_em;
					order_em.clear();
					line_lan.push_back(output_em[i][0]);
					line_lan.push_back(output_em[i][1]);
					line_lan.push_back(output_em[i][2]);
					line_lan.push_back(output_em[i][3]);
					rr=(int)((output_em[i][0]+output_em[i][1]+output_em[i][2])*255);
					gg=(int)((output_em[i][3]+output_em[i][1]+output_em[i][2])*255);
					bb=(int)((output_em[i][0]+output_em[i][3]+output_em[i][2])*255);
					getSortOrderEM(line_lan,order_em,true);
					person[i].group_number=order_em[0];
					person[i].face_color=Scalar(rr,gg,bb);
					groupnumber.push_back(order_em[0]);
					facecolor.push_back(Scalar(rr,gg,bb));
				}
				else
				{
					if(person[i].face_pixPoint.x<psxx)
					{
						person[i].group_number=0;
						groupnumber.push_back(0);
						rr=0;
						gg=0;
						bb=255;
						person[i].face_color=Scalar(rr,gg,bb);
						facecolor.push_back(Scalar(rr,gg,bb));
					}
					else
					{
						person[i].group_number=1;
						groupnumber.push_back(1);
						rr=255;
						gg=0;
						bb=0;
						person[i].face_color=Scalar(rr,gg,bb);
						facecolor.push_back(Scalar(rr,gg,bb));
					}
				}
				circle(cameraImg,person[i].face_pixPoint,3,person[i].face_color,-1,8,0); //face_pixPoint: center of boundingbox
			}
			if(groups_em==1)
			{
				whichem.clear();
				whichem.push_back(0);
				whichem.push_back(1);
				groups_em=2;
			}
			delete em;
	
			/////////////////////////////////////////////////////////////////////
			/////////////////////////cluster information/////////////////////////
			/////////////////////////////////////////////////////////////////////
			Person cluster;
			vector<Person> clusters;
			vector<Object> cluster_body;
			cluster_body.clear();
			Object cluster_f_b;
			int mem_num[4]={0};
			double clu_pos_x[4]={0};
			double clu_pos_y[4]={0};
			double clu_x[4][30]={0};
			double clu_y[4][30]={0};
			double clu_left_x[4]={0};
			double clu_left_y[4]={0};
			double clu_right_x[4]={0};
			double clu_right_y[4]={0};
			double clu_r_x[4]={0};
			double clu_r_y[4]={0};
			double clu_r_z[4]={0};
			vector<double> pos_x;
			vector<double> pos_y;

			for(size_t i=0;i<person.size();i++)
			{
				for(size_t e=0;e<whichem.size();e++)
				{
					if(person[i].group_number==whichem[e])
					{
						clu_pos_x[whichem[e]]=clu_pos_x[whichem[e]]+person[i].face_pixPoint.x;
						clu_pos_y[whichem[e]]=clu_pos_y[whichem[e]]+person[i].face_pixPoint.y;
						clu_x[whichem[e]][mem_num[whichem[e]]]=person[i].face_pixPoint.x;
						clu_y[whichem[e]][mem_num[whichem[e]]]=person[i].face_pixPoint.y;
						clu_r_x[whichem[e]]=clu_r_x[whichem[e]]+person[i].facecenter.x;
						clu_r_y[whichem[e]]=clu_r_y[whichem[e]]+person[i].facecenter.y;
						clu_r_z[whichem[e]]=clu_r_z[whichem[e]]+person[i].facecenter.z;
						mem_num[whichem[e]]++;
					}
				}
			}
			for(size_t e=0;e<whichem.size();e++)
			{
				pos_x.clear();
				pos_y.clear();
				for(size_t t=0;t<mem_num[whichem[e]];t++)
				{
					pos_x.push_back(clu_x[whichem[e]][t]);
					pos_y.push_back(clu_y[whichem[e]][t]);
				}
				vector<size_t> order_clux;
				vector<size_t> order_cluy;
				order_clux.clear();
				order_cluy.clear();
				if(pos_x.size()>1)
				{
					getSortOrderEM(pos_x,order_clux,false);
					getSortOrderEM(pos_y,order_cluy,false);
					if(pos_x[order_clux[0]]>20)
						clu_left_x[whichem[e]]=pos_x[order_clux[0]]-20;
					else
						clu_left_x[whichem[e]]=1;
					if(pos_y[order_cluy[0]]>40)
						clu_left_y[whichem[e]]=pos_y[order_cluy[0]]-40;
					else
						clu_left_y[whichem[e]]=1;
					if(pos_x[order_clux[pos_x.size()-1]]<620)
						clu_right_x[whichem[e]]=pos_x[order_clux[pos_x.size()-1]]+20;
					else
						clu_right_x[whichem[e]]=639;
					if(pos_y[order_cluy[pos_y.size()-1]]<440)
						clu_right_y[whichem[e]]=pos_y[order_cluy[pos_y.size()-1]]+40;
					else
						clu_right_y[whichem[e]]=479;
				}
				if(pos_x.size()==1)
				{
					if(pos_x[0]<40)
					{
						clu_left_x[whichem[e]]=2;
						clu_right_x[whichem[e]]=pos_x[0]+40;
					}
					else if(pos_x[0]>600)
					{
						clu_right_x[whichem[e]]=638;
						clu_left_x[whichem[e]]=pos_x[0]-40;
					}
					else
					{
						clu_left_x[whichem[e]]=pos_x[0]-40;
						clu_right_x[whichem[e]]=pos_x[0]+40;
					}

					if(pos_y[0]<60)
					{
						clu_left_y[whichem[e]]=2;
						clu_right_y[whichem[e]]=pos_y[0]+60;
					}
					else if(pos_y[0]>420)
					{
						clu_right_y[whichem[e]]=478;
						clu_left_y[whichem[e]]=pos_y[0]-60;
					}
					else
					{
						clu_left_y[whichem[e]]=pos_y[0]-60;
						clu_right_y[whichem[e]]=pos_y[0]+60;
					}


				}


			}
		
			/////////////////////////////////////////////////////////////////////
			////////////////////////cluster information//////////////////////////
			/////////////////////////////////////////////////////////////////////
			clusters.clear();
			for(size_t lu=0;lu<4;lu++)
			{
				if(mem_num[lu]!=0)
				{
					clu_pos_x[lu]=(double)(clu_pos_x[lu]/mem_num[lu]*1.0);
					clu_pos_y[lu]=(double)(clu_pos_y[lu]/mem_num[lu]*1.0);
					clu_r_x[lu]=clu_r_x[lu]/mem_num[lu];
					clu_r_y[lu]=clu_r_y[lu]/mem_num[lu];
					clu_r_z[lu]=clu_r_z[lu]/mem_num[lu];

					cluster.face_pixPoint.x=(int)(clu_pos_x[lu]);
					cluster.face_pixPoint.y=(int)(clu_pos_y[lu]);
					cluster.face_leftup.x=(int)(clu_left_x[lu]);
					cluster.face_leftup.y=(int)(clu_left_y[lu]);
					cluster.face_width=(int)(clu_right_x[lu]-clu_left_x[lu]);
					cluster.face_height=(int)(clu_right_y[lu]-clu_left_y[lu]);
					cluster.facecenter.x=clu_r_x[lu];
					cluster.facecenter.y=clu_r_y[lu];
					cluster.facecenter.z=clu_r_z[lu];
					cluster.rectangle.x=cluster.face_leftup.x;
					cluster.rectangle.y=cluster.face_leftup.y;
					cluster.rectangle.width=cluster.face_width;
					cluster.rectangle.height=cluster.face_height;
					cluster.weight=1;
					cluster.group_number=lu;
					clusters.push_back(cluster);
					cluster_f_b.pixPoint.x=cluster.face_pixPoint.x;
					cluster_f_b.pixPoint.y=cluster.face_pixPoint.y;
					cluster_f_b.boundingbox=cluster.rectangle;
					cluster_body.push_back(cluster_f_b);
				}
			}
			vector<double> testclu;
			vector<size_t> testclor;
			for(size_t cl=0;cl<clusters.size();cl++)
			{
				testclu.push_back(clusters[cl].face_pixPoint.x);
			}
			getSortOrder(testclu,testclor,true);
			vector<Person> people1;////
			people1.clear();
			for(int i=0; i<clusters.size();i++)
			{
				people1.push_back(clusters[testclor[i]]);
			}
			
			//////////////////////////////composition evaluation /////////////////////////
			double kld_ls;
			calKLD_multi2(cluster_body,kld_ls);
			if(kld_ls<0)
				kld_ls=0;
			////////////////////////////record evalution result to the rgb image///////////////// 
			ostringstream information1;
			information1.precision(3);
			information1<<kld_ls;
			cv::putText(cameraImg, "KLD_C: " + information1.str(), cv::Point(25,25), CV_FONT_NORMAL, 0.75, Scalar(255,50,50),1,1);
			///////////////////////save the images as a avi video////////////////////////////////
			//writer1<<cameraImg;
			///////////////////////show the rgb image ///////////////////////////
			//imshow("result",cameraImg);
			
			waitKey(3);
			if(groups_em>1)
			{
				
				
				
				////////////////////////////////////楕円の描写はクラスタ数が２の場合のみ////////////////////////////////////
				if(clusters.size()==2)
				{
					
					/////////////////present image について////////////////////////
					vector<Point> point_cluster;
					point_cluster.clear();
					Point cluster_p;
					for(size_t i=0;i<clusters.size();i++)
					{
						cluster_p.x=clusters[i].face_pixPoint.x;
						cluster_p.y=clusters[i].face_pixPoint.y;
						point_cluster.push_back(cluster_p);
					}
					Point cluster_p1,cluster_p2;
					cluster_p1.x=clusters[0].face_pixPoint.x;
					cluster_p1.y=clusters[0].face_pixPoint.y;
					cluster_p2.x=clusters[1].face_pixPoint.x;
					cluster_p2.y=clusters[1].face_pixPoint.y;
					point_cluster.push_back(cluster_p1);
					point_cluster.push_back(cluster_p2);
					
					int angle=0;
					if(sizess_lan.size()==2) 
					{
						for(size_t i=0;i<centerss_lan.size();i++)
						{
							ellipse(cameraImg,centerss_lan[i],Size(sizess_lan[i].x,sizess_lan[i].y),angless_lan[i],angless_lan[i],angless_lan[i]+360,Scalar(255,200,100),3,4,0);
						}
					}
					else
					{
						double sum_center_l=0;
						double sum_center_ly=0;
						int counter_l=0;
						int counter_r=0;
						double sum_center_r=0;
						double sum_center_ry=0;
						double xiefangl=0;
						double xiefangr=0;
						for(size_t j=0;j<person.size();j++)
						{
							if(person[j].face_pixPoint.x<psxx)
							{
								sum_center_l=sum_center_l+person[j].face_pixPoint.x;
								sum_center_ly=sum_center_ly+person[j].face_pixPoint.y;
								xiefangl=xiefangl+person[j].face_pixPoint.x*person[j].face_pixPoint.y;
								counter_l++;
							}
							else
							{
								sum_center_r=sum_center_r+person[j].face_pixPoint.x;
								sum_center_ry=sum_center_ry+person[j].face_pixPoint.y;
								xiefangr=xiefangr+person[j].face_pixPoint.x*person[j].face_pixPoint.y;
								counter_r++;
							}
						}
						/////////////////mean///////////////////////////
						sum_center_l=sum_center_l/counter_l;
						sum_center_r=sum_center_r/counter_r;
						sum_center_ly=sum_center_ly/counter_l;
						sum_center_ry=sum_center_ry/counter_r;
						// foutpinglx<<sum_center_l<<endl;
						// foutpingly<<sum_center_ly<<endl;
						// foutpingrx<<sum_center_r<<endl;
						// foutpingry<<sum_center_ry<<endl;

						int xleft,xright,yleft,yright;
						xleft=(int)(sum_center_l);
						yleft=(int)(sum_center_ly);
						xright=(int)(sum_center_r);
						yright=(int)(sum_center_ry);
						xiefangl=xiefangl/counter_l;
						xiefangr=xiefangr/counter_r;

						xiefangl=xiefangl-sum_center_l*sum_center_ly;
						xiefangr=xiefangr-sum_center_r*sum_center_ry;
						// foutxiel<<xiefangl<<endl;
						// foutxier<<xiefangr<<endl;

						double varlx=0;
						double varly=0;
						double varrx=0;
						double varry=0;

					for(size_t j=0;j<person.size();j++){
						if(person[j].face_pixPoint.x<psxx)
						{
							varlx=varlx+(person[j].face_pixPoint.x-sum_center_l)*(person[j].face_pixPoint.x-sum_center_l);
							varly=varly+(person[j].face_pixPoint.y-sum_center_ly)*(person[j].face_pixPoint.y-sum_center_ly);
						}
						else
						{
							varrx=varrx+(person[j].face_pixPoint.x-sum_center_r)*(person[j].face_pixPoint.x-sum_center_r);
							varry=varry+(person[j].face_pixPoint.y-sum_center_ry)*(person[j].face_pixPoint.y-sum_center_ry);
						}
					}
					varlx=varlx/counter_l;
					varly=varly/counter_l;
					varrx=varrx/counter_r;
					varry=varry/counter_r;

					double stdlx,stdly,stdrx,stdry;
					stdlx=sqrt(varlx);
					stdly=sqrt(varly);
					stdrx=sqrt(varrx);
					stdry=sqrt(varry);
					// foutbiaolx<<stdlx<<endl;
					// foutbiaoly<<stdly<<endl;
					// foutbiaorx<<stdrx<<endl;
					// foutbiaory<<stdry<<endl;

					double cosl=0;
					double cosrk=0;
					int anglel;
					int anglerk;
					cosl=xiefangl/(stdlx*stdly);
					cosl=acos(cosl);
					cosl=cosl/3.14*180.0;
					anglel=(int)(cosl);
					cosrk=xiefangr/(stdrx*stdry);
					cosrk=acos(cosrk);
					cosrk=cosrk/3.14*180.0;
					anglerk=(int)(cosrk);
					// ostringstream ostime;
					// ostime.precision(3);
					// ostime<<anglerk;

					//putText(colorImagelk,"angle:"+ostime.str(),Point(10,40),0,1,Scalar(0,0,0),2,CV_AA);

					Point psizel;
					Point psizerk;
					psizel.x=(int)(stdlx);
					psizel.y=(int)(stdly);
					psizerk.x=(int)(stdrx);
					psizerk.y=(int)(stdry);
				
					ellipse(cameraImg,cluster_p1,Size(psizel.x*2,psizel.y*2),anglel,anglel,anglel+360,Scalar(255,200,100),3,4,0);
					ellipse(cameraImg,cluster_p2,Size(psizerk.x*2,psizerk.y*2),anglerk,anglerk,anglerk+360,Scalar(255,200,100),3,4,0);
					//ellipse(colorImagelk,cluster_p1,Size(psizel.x*2,psizel.y*2),-anglel+90,anglel,anglel+360,Scalar(255,200,100),3,4,0);
					//ellipse(colorImagelk,cluster_p2,Size(psizerk.x*2,psizerk.y*2),anglel,anglel,anglel+360,Scalar(255,200,100),3,4,0);
					}
				}
				else                 ////////////cluster数は2以上/////////////////////
				{
					for(size_t i=0;i<centerss_lan.size();i++)
					{
						ellipse(cameraImg,centerss_lan[i],Size(sizess_lan[i].x,sizess_lan[i].y),angless_lan[i],angless_lan[i],angless_lan[i]+360,Scalar(255,200,100),3,4,0);
					}
				}
				///////////////////////save the images as a avi video////////////////////////////////
				writer1<<cameraImg;
				///////////////////////show the rgb image ///////////////////////////
				imshow("result",cameraImg);
			}
	
			
		}
		
	}

		}
	}catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
					//ros_duration = ros::Time::now().toSec()-ros_begin;
					//fout_data_time<<ros_duration<<endl;//record the time
}

int main(int argc, char **argv){
    ros::init(argc, argv, "depth_estimater");
	ROS_INFO("main started");
	fout_data_x.open(file_data_x.c_str());
  	fout_data_y.open(file_data_y.c_str());
  	fout_data_z.open(file_data_z.c_str());
	fout_data_yaw.open(file_data_yaw.c_str());
    fout_data_differ_x.open(file_data_differ_x.c_str());
    fout_data_differ_y.open(file_data_differ_y.c_str());
    fout_data_bboxheight.open(file_data_bboxheight.c_str());
    fout_data_time.open(file_data_time.c_str());
	fout_data_score.open(file_data_score.c_str());
    depth_estimater depth_estimater;
	ros_begin = ros::Time::now().toSec();
    while (ros::ok())
    {
	ros::spinOnce();
    	if(cv::waitKey(1)=='q')
    	{
    		ros::shutdown();
    	}
    }
  fout_data_x.close();
  fout_data_y.close();
  fout_data_z.close();
  fout_data_yaw.close();	
  fout_data_differ_x.close();
  fout_data_differ_y.close();
  fout_data_bboxheight.close();
  fout_data_time.close();
  fout_data_score.close();
  writer1.release();
  writer2.release();
  writer3.release();
    return 0;
}
