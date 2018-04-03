#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sophus/sim3.hpp"

#include "map_accum.h"

#include "std_msgs/String.h"
#include <sstream>





mapAccumulator* mapAcc = 0;

int main(int argc, char **argv)
{
  printf("Starting a node for extracting map from LSD SLAM\n");

  printf("TESTING\n");

  ros::init(argc, argv, "lsd_slam_map");


  mapAcc = new mapAccumulator();

  printf("created class\n");

  ros::Rate loopRate(1);



  /*
  while(ros::ok())
  {
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world ";
  //  msg.data = ss.str();

    //mapAcc->pub.publish(msg);

    ros::spinOnce();

    loopRate.sleep();
  }

*/ros::spin();


}
