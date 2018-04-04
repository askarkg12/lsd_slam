#pragma once

#include "ros/ros.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"

#include "std_msgs/Empty.h"


#include "std_msgs/String.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"

#include "KeyFrameDisplay.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_ros/pcl_nodelet.h>

#include "sophus/sim3.hpp"
#include "KeyFrameGraphDisplay.h"



class mapAccumulator
{
  public:
    mapAccumulator();
    ~mapAccumulator();

    ros::NodeHandle nh;

    ros::Publisher pub;

    ros::Subscriber frameSub, graphSub, requestSub, liveSub;






  private:
    void callbackTest(lsd_slam_viewer::keyframeMsgConstPtr msg);
    void reset();
    void KeyFrameCallback(lsd_slam_viewer::keyframeMsgConstPtr msg);
    void LiveFrameCallback(lsd_slam_viewer::keyframeMsgConstPtr msg);
    void GraphCallback(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);
    void RequestCallback(std_msgs::Empty msg);

    float scaleMult;

		InputPointDense* inputPoints;
		Sophus::Sim3f camToWorld;
    int latestFrameID;

    std::map<int, lsd_slam_viewer::keyframeMsg> keyFrames;

};
