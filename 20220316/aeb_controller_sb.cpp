#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

std_msgs::Bool flag_AEB;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
  ROS_INFO("Sonar Range: [%f]", msg->range);
  
  
	if(msg->range <=1.0)
	{
		ROS_INFO("AEB_Activation!!");
		flag_AEB.data=true;
	}
	else
	{
		flag_AEB.data =false;
	}
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "infrared_listener");
  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("sensor/sonar0", 1000, UltraSonarCallback);
  
  ros::spin();

  return 0;
}
