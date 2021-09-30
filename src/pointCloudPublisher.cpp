#include "vision_node_incl.h"

unsigned indexVec = 0;
unsigned indexGps = 0;
std::vector<sensor_msgs::PointCloud2> messages;
std::vector<sensor_msgs::NavSatFix> gpsData;
ros::Publisher pub;
ros::Publisher pubGps;

void addMessagesToVector(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  messages.push_back(*cloud_msg);
}

void addGpsDataToVector(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
  gpsData.push_back(*gps_msg);
}

void publishData()
{
  if (indexVec <= messages.size()) {
    pub.publish(messages[indexVec]);}
  indexVec++;
  if (indexGps <= gpsData.size()) {
    pubGps.publish(gpsData[indexGps]);}
  indexGps++;
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "pointCloudPublisher");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1000, addMessagesToVector);
    pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    ros::Subscriber subGps = nh.subscribe<sensor_msgs::NavSatFix> ("gps/filtered", 1000, addGpsDataToVector);
    pubGps = nh.advertise<sensor_msgs::NavSatFix>("gps_data", 1);
    ros::Timer timerPub = nh.createTimer(ros::Duration(1.0 / 2.0), std::bind(publishData));
    ros::spin(); 
  
    return 0;
}