#include "map_accum.h"

#include "ros/ros.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"

#include "std_msgs/Empty.h"

#include "std_msgs/Float32.h"

#include "sensor_msgs/PointCloud2.h"

#include "KeyFrameDisplay.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_ros/pcl_nodelet.h>

#include "sophus/sim3.hpp"
#include "KeyFrameGraphDisplay.h"

#include <tf/transform_broadcaster.h>

#include <tf/transform_listener.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <math.h>

mapAccumulator::mapAccumulator()
{
  pub = nh.advertise<sensor_msgs::PointCloud2>("/lsd_slam/pointcloud2", 1);

  scaleMult = 4;

  //subscribe to keyframe topics
  frameSub = nh.subscribe(nh.resolveName("/lsd_slam/keyframes"),20, &mapAccumulator::KeyFrameCallback, this);
  liveSub = nh.subscribe(nh.resolveName("/lsd_slam/liveframes"),20, &mapAccumulator::LiveFrameCallback, this);
  graphSub = nh.subscribe(nh.resolveName("lsd_slam/graph"), 5, &mapAccumulator::GraphCallback, this);
  requestSub = nh.subscribe(nh.resolveName("lsd_slam/map_request"), 1, &mapAccumulator::RequestCallback, this);
  scaleSub = nh.subscribe(nh.resolveName("/lsd_slam/map_scale"), 1, &mapAccumulator::ScaleCallback, this);
  latestFrameID = 0;

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  //bool flag = true;
  while(true)
  {
  try
  {
    slam_tf = tfBuffer.lookupTransform("nav", "ardrone_base_frontcam",     ros::Time(0));
    ROS_INFO("It worked");
    break;
  }
  catch (tf2::TransformException ex)
  {
     //ROS_ERROR("%s",ex.what());
   }
 }

  printf("%f %f %f\n", slam_tf.transform.translation.x, slam_tf.transform.translation.y, slam_tf.transform.translation.z);
  slam_tf.header.stamp = ros::Time(0);
  slam_tf.header.frame_id = "nav";
  slam_tf.child_frame_id = "slam_tf";

  //slam_tf.transform.rotation.w = 1;
  static_broadcaster.sendTransform(slam_tf);

  cam_tf.child_frame_id = "cam_tf";
  cam_tf.header.frame_id = "slam_tf";
  cam_tf.transform.rotation.x = 0;
  cam_tf.transform.rotation.x = 0;
  cam_tf.transform.rotation.x = 0;
  cam_tf.transform.rotation.w = 1;

}

void mapAccumulator::ScaleCallback(std_msgs::Empty msg)
{
  geometry_msgs::TransformStamped drone, cam;
  while(true)
  {
    try
    {
      drone = tfBuffer.lookupTransform("slam_tf", "ardrone_base_frontcam",     ros::Time(0));
      ROS_INFO("It worked");
      break;
    }
    catch (tf2::TransformException ex)
    {
    }
  }
  while(true)
  {
    try
    {
      cam = tfBuffer.lookupTransform("slam_tf", "cam_tf",     ros::Time(0));
      ROS_INFO("It worked");
      break;
    }
    catch (tf2::TransformException ex)
    {
    }
  }
  float lenCam, lenDrone;
  lenCam = sqrt(pow(cam.transform.translation.x, 2) + pow(cam.transform.translation.z, 2));
  lenDrone = sqrt(pow(drone.transform.translation.x, 2) + pow(drone.transform.translation.z, 2));
  float ratio = lenDrone/lenCam;
  printf("Current ratio is %f\n", ratio);
  scaleMult = scaleMult*ratio;
  printf("New scale is %f\n", scaleMult);
}

void mapAccumulator::LiveFrameCallback(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
  memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
  Sophus::Vector3f vec = camToWorld.translation();
  cam_tf.transform.translation.x = vec[0]*scaleMult;
  cam_tf.transform.translation.y = vec[1]*scaleMult;
  cam_tf.transform.translation.z = vec[2]*scaleMult;
  cam_tf.header.stamp = ros::Time(0);
  broadcaster.sendTransform(cam_tf);
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

      printf("Scale is %f\n", camToWorld.scale()*3);

      for (int y = 2; y < height - 2; y++)
      {

        for (int x = 2; x < width - 2; x++)
        {
          if (x == 0 || y == 0 || x == width - 1 || y == height - 1)
          {
            continue;
          }

          if (inputPoints[x + y * width].idepth <= 0)
          {
            continue;
          }

          int nearSupport = 0;

          for (int dx = -2; dx <3; ++dx)
          {
            for (int dy = -2; dy<3; ++dy)
            {
              const InputPointDense pointNear = inputPoints[x+dx+(y+dy)*width];

              if (pointNear.idepth > 0)
              {
                const float diff = pointNear.idepth - inputPoints[x + y * width].idepth;
                if (diff * diff < 2 * inputPoints[x + y * width].idepth_var)
                {
                  nearSupport++;
                }
              }
            }
          }

          if (nearSupport < 21)
          {
            continue;
          }

          float depth = 1 / inputPoints[x + y * width].idepth;
          Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x * fxi + cxi), (y * fyi + cyi), 1) * depth);
          pcl::PointXYZ point;
          point.x = scaleMult*pt[0];
          point.y = scaleMult*pt[1];
          point.z = scaleMult*pt[2];

          cloud_pcl.push_back(point);
          count++;
        }
      }
    }
  }
  cloud_pcl.width = count;
  cloud_pcl.height = 1;
  cloud_pcl.is_dense = false;
  cloud_pcl.header.frame_id = "slam_tf";
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
    cloud_pcl.header.frame_id = "slam_tf";
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
