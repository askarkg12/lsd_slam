#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sophus/sim3.hpp"

int main(int argc, char **argv)
{
  printf("Starting a node for extracting map from LSD SLAM\n");

  ros::init(argc, argv, "lsd_slam_map");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Empty>("testTopic", 1);

  ros::Rate loopRate(1);

  while(ros::ok())
  {
    std_msgs::Empty msg;
    pub.publish(msg);

    ros::spinOnce();

    loopRate.sleep();
  }

}
/*
mapExtractor::mapExtractor()
{

}
*/
