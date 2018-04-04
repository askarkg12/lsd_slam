#include "map_accum.h"

#include "ros/ros.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

#include "sensor_msgs/PointCloud2.h"

#include "KeyFrameDisplay.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_ros/pcl_nodelet.h>

#include "sophus/sim3.hpp"
#include "KeyFrameGraphDisplay.h"

mapAccumulator::mapAccumulator()
{
  printf("This is happening from the class");



  pub = nh.advertise<sensor_msgs::PointCloud2>("testTopic", 1);

  scaleMult = 2;

  //subscribe to keyframe topics
  frameSub = nh.subscribe(nh.resolveName("/lsd_slam/keyframes"),20, &mapAccumulator::KeyFrameCallback, this);
  liveSub = nh.subscribe(nh.resolveName("/lsd_slam/liveframes"),20, &mapAccumulator::LiveFrameCallback, this);
  graphSub = nh.subscribe(nh.resolveName("lsd_slam/graph"), 5, &mapAccumulator::GraphCallback, this);
  requestSub = nh.subscribe(nh.resolveName("lsd_slam/map_request"), 1, &mapAccumulator::RequestCallback, this);
  latestFrameID = 0;
}

void mapAccumulator::LiveFrameCallback(lsd_slam_viewer::keyframeMsgConstPtr msg)
{

}

void mapAccumulator::RequestCallback(std_msgs::Empty msg)
{
  printf("Map has been requested\n");
  std::map<int, lsd_slam_viewer::keyframeMsg>::iterator Kf;
  pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
  int count = 0;
  int frameCount = 0;
  for (Kf = keyFrames.begin(); Kf != keyFrames.end(); Kf++)
  {
    if (Kf->second.isKeyframe)
    {

      const float fxi = 1 / Kf->second.fx;
      const float fyi = 1 / Kf->second.fy;
      const float cxi = -Kf->second.cx / Kf->second.fx;
      const float cyi = -Kf->second.cy / Kf->second.fy;

      const int width = Kf->second.width;
      const int height = Kf->second.height;

      inputPoints = new InputPointDense[width * height];

      memcpy(camToWorld.data(), Kf->second.camToWorld.data(), 7*sizeof(float));
      memcpy(inputPoints, Kf->second.pointcloud.data(), width*height*sizeof(InputPointDense));

      //float a = camToWorld.scale();
      //float b = a*scaleMult;
      //camToWorld.setScale(b);

      printf("Scale is %f\n", camToWorld.scale());

      for (int y = 1; y < height - 1; y++)
      {

        for (int x = 1; x < width - 1; x++)
        {
          if (x == 0 || y == 0 || x == width - 1 || y == height - 1)
          {
            continue;
          }

          if (inputPoints[x + y * width].idepth <= 0)
          {
            continue;
          }

          float depth = 1 / inputPoints[x + y * width].idepth;
          Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x * fxi + cxi), (y * fyi + cyi), 1) * depth);
          pcl::PointXYZ point;
          point.x = pt[0];
          point.y = pt[1];
          point.z = pt[2];

          cloud_pcl.push_back(point);
          count++;
        }
      }
    }
  }
  cloud_pcl.width = count;
  cloud_pcl.height = 1;
  cloud_pcl.is_dense = false;
  cloud_pcl.header.frame_id = "nav";
  pcl_conversions::toPCL(ros::Time::now(), cloud_pcl.header.stamp);

  sensor_msgs::PointCloud2 mes;
  pcl::toROSMsg(cloud_pcl, mes);

  pub.publish(mes);
  printf("Map construncted from %d keyFrames\nFinished extracting total map. Total of %d points\n", frameCount, count);
}

void mapAccumulator::callbackTest(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
  if (msg->isKeyframe)
  {
    printf("Frame id is : %d\n", msg->id);
    printf("Received a keyframe\n");
    pcl::PointCloud<pcl::PointXYZ> cloud_pcl;

    const float fxi = 1 / msg->fx;
    const float fyi = 1 / msg->fy;
    const float cxi = -msg->cx / msg->fx;
    const float cyi = -msg->cy / msg->fy;
    printf("a\n");
    const int width = msg->width;
    const int height = msg->height;
    printf("b\n");
    int count = 0;
    printf("c\n");
    inputPoints = new InputPointDense[width * height];
    memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
    memcpy(inputPoints, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
    printf("1");
    for (int y = 1; y < height - 1; y++)
    {

      for (int x = 1; x < width - 1; x++)
      {
        if (x == 0 || y == 0 || x == width - 1 || y == height - 1)
        {
          continue;
        }

        if (inputPoints[x + y * width].idepth <= 0)
        {
          continue;
        }

        float depth = 1 / inputPoints[x + y * width].idepth;
        Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x * fxi + cxi), (y * fyi + cyi), 1) * depth);
        pcl::PointXYZ point;
        point.x = pt[0];
        point.y = pt[1];
        point.z = pt[2];

        cloud_pcl.push_back(point);
        count++;
      }
    }
    printf("2");

    cloud_pcl.width = count;
    cloud_pcl.height = 1;
    cloud_pcl.is_dense = false;
    cloud_pcl.header.frame_id = "world";
    pcl_conversions::toPCL(ros::Time::now(), cloud_pcl.header.stamp);

    sensor_msgs::PointCloud2 mes;
    pcl::toROSMsg(cloud_pcl, mes);

    pub.publish(mes);
    printf("Finished\n");
  }
}

void mapAccumulator::GraphCallback(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{

  GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
  int ErrorCount = 0;
  int numGraphPoses = msg->numFrames;
  for (int i = 0; i < numGraphPoses; i++)
  {
    if (keyFrames.count(graphPoses[i].id) == 0)
    {
      ErrorCount++;
    }
    else
    {
      memcpy(keyFrames[graphPoses[i].id].camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
    }
  }
  printf("Number of frames - %d\nErrorCount is - %d\n", numGraphPoses, ErrorCount);
}

void mapAccumulator::KeyFrameCallback(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
  if (msg->id == 1)
  {
    printf("Started from zero, deleting all the saved frames\n");
    keyFrames.clear();
  }
  latestFrameID = msg->id;
  lsd_slam_viewer::keyframeMsg temp = *msg;
  keyFrames[temp.id] = temp;
  printf("Latest frmae ID - %d, Current one - %d\n", latestFrameID, temp.id);
  printf("Times is - %f\n\n", temp.time);
  printf("Current size of the buffer: %d\n", keyFrames.size());
}

void mapAccumulator::reset()
{
  keyFrames.clear();
}
