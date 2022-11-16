#include "ros/ros.h"
//#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;
//using namespace Eigen;

//uwb stands for mocap here
float pos_uwb[3]={0,0,0};
float q_uwb[4]={0,0,0,0};

string mocap_unit="m";

ros::Publisher vision_pub;

void uwb_to_fcu();

void cb_uwb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped uwb_data;
	uwb_data = *msg;
	if(mocap_unit == "m")
	{
		pos_uwb[0] =  uwb_data.pose.position.x;//m to m
		pos_uwb[1] =  uwb_data.pose.position.y;
		pos_uwb[2] =  uwb_data.pose.position.z;
	}
	else if(mocap_unit == "mm")
	{
		pos_uwb[0] =  uwb_data.pose.position.x/1000;//mm to m
		pos_uwb[1] =  uwb_data.pose.position.y/1000;
		pos_uwb[2] =  uwb_data.pose.position.z/1000;	
	}
	else
	{
		ROS_WARN("[px4mocap] invalid mocap_unit for px4mocap!");
	}
	q_uwb[0] = uwb_data.pose.orientation.x;
	q_uwb[1] = uwb_data.pose.orientation.y;
	q_uwb[2] = uwb_data.pose.orientation.z;
	q_uwb[3] = uwb_data.pose.orientation.w;
}


void uwb_to_fcu()
{
	geometry_msgs::PoseStamped vision;

        vision.pose.position.x = pos_uwb[0];
        vision.pose.position.y = pos_uwb[1];
        vision.pose.position.z = pos_uwb[2];

        vision.pose.orientation.x = q_uwb[0];
        vision.pose.orientation.y = q_uwb[1];
        vision.pose.orientation.z = q_uwb[2];
        vision.pose.orientation.w = q_uwb[3];

	vision.header.stamp = ros::Time::now();
	vision_pub.publish(vision);
	
}



int main(int argc,char **argv)
{
	ros::init(argc,argv,"px4mocap0");
	ros::NodeHandle n("");
	ros::NodeHandle nh("~");
	
	n.param<string>("mocap_unit", mocap_unit, "m");
	
	ros::Subscriber uwb_sub = n.subscribe("rigidpose_topic",10,cb_uwb);
	vision_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

	ros::Rate loop_rate(60); //50Hz 30-50Hz is suitable

	int count = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		if( (pos_uwb[0]==0)&&(pos_uwb[1]==0) ) 
		{	if(count == 1) {ROS_INFO("[px4mocap] no data from mocap!");}
		}
		else if( (pos_uwb[0]>900)&&(pos_uwb[1]>900) ) 
		{
			if(count == 1) {ROS_INFO("[px4mocap] mocap pos too large! Maybe lost the track.");}
		}
		else
		{
			uwb_to_fcu();
		}
		loop_rate.sleep();
		
		count = count + 1;
		if(count==180) {count = 0;}
	
	}  //无法在回调函数里发布话题，报错函数里没有定义vel_pub!只能在main里面发布了

	return 0;
}
