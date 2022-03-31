#include "ros/ros.h"
#include <sensor_msgs/Range.h>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

sensor_msgs::Range avg_range;

float range1,range2,range3,range4,range5,range_avg;

void RangerCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    range1 = range2 = range3 = range4 = range5 = msg->range;
    range_avg = (range1 + range2 + range3 + range4 + range5) /5.0;
    cout << "" << range_avg << endl;   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv , "sonar_avg_filter");
  
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/range",1000,RangerCallback);
  
  ros::Publisher pub_range_avg = n.advertise<sensor_msgs::Range>("/range_avg",1000);
  
  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
     pub_range_avg.publish(avg_range);
     
     loop_rate.sleep();
     ros::spinOnce();
  
  }
  return 0;
  
}
