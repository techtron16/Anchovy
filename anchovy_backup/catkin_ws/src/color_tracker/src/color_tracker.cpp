#include <cstdio>
#include <stdlib.h>
#include <cmath>
#include <boost/config/no_tr1/complex.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmvision/Blobs.h>
#include <cmvision/Blob.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

class colorTracker
{
  private:
    geometry_msgs::Vector3 trackedObject;
    sensor_msgs::PointCloud2 depth;
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    
    tf::Transform colorTF;

    tf::TransformListener listenerTF;
    tf::TransformBroadcaster br;
    
    ros::NodeHandle *n;
    
  public:
    ros::Publisher blobPub;
    
    void blobCallback(const cmvision::Blobs::ConstPtr& msg)
    {
      cmvision::Blob colorObj;
      cmvision::Blob blob;

      if(msg->blob_count==0)
      {
	ROS_INFO("No blobs detected.");
	return;
      }

      blob = msg->blobs[0];

      colorObj = blob;

      printf("x: %u, y: %u, area: %u\n", blob.x, blob.y, blob.area);
      
      geometry_msgs::Vector3 blobby;
      blobby.x = int(blob.x) - 320;
      blobby.y = 480 - blob.y;
      blobby.z = blob.area;
      
      blobPub.publish<geometry_msgs::Vector3>(blobby);
      
      trackedObject = getPose(blob.x, blob.y);
      if (trackedObject.z == trackedObject.z)
      {
           cout << trackedObject.x << " : " << trackedObject.y << " : " << trackedObject.z << endl;
           /*printf("Position of the tracked object >>> x: %f; y: %f; z: %f\n", 
	     	trackedObject.linear.x, trackedObject.linear.y, trackedObject.linear.z);*/
           tfPublisher();
      }
    }
    
    geometry_msgs::Vector3 getPose(int x, int y)
    {
      geometry_msgs::Vector3 v;
      pcl::PointXYZ point = pcl_cloud.at(x, y);
      v.x = point.x;
      v.y = point.y;
      v.z = point.z;
      return v;
    }
    
    void tfPublisher()
    {
      colorTF.setOrigin( tf::Vector3(trackedObject.z, -trackedObject.x, -trackedObject.y));
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      colorTF.setRotation(q);
      
      br.sendTransform(tf::StampedTransform(colorTF, ros::Time::now(), "/camera_link", "/colorObj"));
    }

    void kinectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
    {
      depth = *msg;

      //std::cout<<msg->height<<" ";
      pcl::PCLPointCloud2 pcl_pc2;
      //pcl_conversions::toPCL((sensor_msgs::PointCloud2&)msg, pcl_pc2);
      pcl_conversions::toPCL(*msg, pcl_pc2);
       //Actually convert the PointCloud2 message into a type we can reason about
      pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    }
    
    void setNodeHandle(ros::NodeHandle* nh)
    {
      n = nh;
      pcl_cloud.width = 640;
      pcl_cloud.height = 480;
      pcl_cloud.resize(640 * 480);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_tracker");
  ros::NodeHandle n;
  
  colorTracker tracker;
  
  tracker.setNodeHandle(&n);
  
  tracker.blobPub = n.advertise<geometry_msgs::Vector3>("/blobPt", 15);
  
  ros::Subscriber kinectSub = n.subscribe("/minnow/camera/depth/points", 1, &colorTracker::kinectDepthCallback, &tracker);
  ros::Subscriber blobSub = n.subscribe("/blobs", 1000, &colorTracker::blobCallback, &tracker);
  
  ros::spin();
}
