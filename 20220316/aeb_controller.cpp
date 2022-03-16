 #include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char **argv)
{
	int count =0;
	
	ros :: init(argc, argv, "aeb_controller");
	
	ros ::NodeHandle n;
	
	ros ::Rate loop_rate(2);
	
	ros ::Subscriber sub = n.subscribe("range", 1000);
	
	while(ros::ok())
	{   
		sensor_msgs::Range msg;
		pub.publish(msg);
		
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}

