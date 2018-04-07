#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sophus/sim3.hpp"

#include "map_accum.h"

#include "std_msgs/String.h"
#include <sstream>

#include <tf/transform_broadcaster.h>





mapAccumulator* mapAcc = 0;

int main(int argc, char **argv)
{
  printf("Starting a node for extracting map from LSD SLAM\n");

  printf("TESTING\n");

  ros::init(argc, argv, "lsd_slam_map");


  mapAcc = new mapAccumulator();

  //ros::Rate loopRate(10.0);


  ros::spin();


}
