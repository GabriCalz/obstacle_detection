#include "vision_node_incl.h"

//Definition of some useful parameters
const float ELEVATION_THRESHOLD = 0.1f;
const float SLOPE_THRESHOLD = 0.5f; //0.15
const double ELEVATION_SCALE = 0.25;
const int MAXIMUM_WINDOW_SIZE = 18;
const int SEARCH_AREA_LIMITS [] = {-10, 10, -10, 10, -2, 0};
const float MIN_DISTANCE_CLUSTERS = 1.0f; //Min distance between points from two clusters
const int MIN_POINTS_CLUSTERS = 12; //Set the minimum number of points per cluster
const int MAX_POINTS_CLUSTERS = 10000; //Set the maximum number of points per cluster
const float ROVER_RADIUS = 2.5f;
const float LEAF_SIZE = 0.2f;
float CELL_SIZE = 0.1f;
const float OBS_DIST_THRESHOLD = 3.0f;
ros::Publisher occupancy_pub;
sensor_msgs::NavSatFix current_gps;

const int TYPE_OF_OCCUPANCY_GRID = 1; // = 0 -> the size of the occupancy grid cell is about 0.1
                                      // = 1 -> the size of the occupancy grid cell is about 1.0
                                      // = 2 -> the sizes of the obstacles are defined by 
                                      // rectangular boundaries

void segmentationOfEnvironment (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ros::Time startTime = ros::Time::now(); //Get the start time
  if (TYPE_OF_OCCUPANCY_GRID == 1) {CELL_SIZE = 1.0f;}
  std::cout << "-----------------------------------------------" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr MyPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);
  pcl::fromROSMsg(*cloud_msg, *MyPointCloud);
  
  std::cout << "The Pointcloud is successfully loaded. Original dimension: "
            << MyPointCloud->size()
            << " data points."
            << std::endl;
  std::cout << "GPS Location -> Latitude: " << current_gps.latitude  << "| " 
                              "Longitude: " << current_gps.longitude << "| " 
                               "Altitude: " << current_gps.altitude  << std::endl;
  int degreesLat = int(floor(current_gps.latitude));
  int degreesLon = int(floor(current_gps.longitude));
  int minutesLat = (int) ( (current_gps.latitude - (float)degreesLat) * 60.f);
  int minutesLon = (int) ( (current_gps.longitude - (float)degreesLon) * 60.f);
  int secondsLat = (int) ( (current_gps.latitude - (float)degreesLat - (float)minutesLat / 60.f) * 60.f * 60.f );
  int secondsLon = (int) ( (current_gps.longitude - (float)degreesLon - (float)minutesLon / 60.f) * 60.f * 60.f );
  char noso = (current_gps.latitude > 0) ? 'N' : 'S'; 
  char eswe = (current_gps.longitude > 0) ? 'E' : 'W'; 
  std::cout << "GPS Location -> " << degreesLat << "°" << noso << " " 
                                  << minutesLat << "' " << secondsLat << "'' | "
                                  << degreesLon << "°" << eswe << " "
                                  << minutesLon << "' " << secondsLon << "''" << std::endl;

  // Filter the point cloud using voxel grid
  pcl::VoxelGrid<pcl::PointXYZ> filterVoxGrid;
  filterVoxGrid.setInputCloud(MyPointCloud);
  filterVoxGrid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  filterVoxGrid.filter(*MyPointCloud);

  std::cout << "Execution Steps"  << std::endl;
  std::cout << "Filtering.. Number of Points now: " << MyPointCloud->size() << std::endl;

  //Code to delete from the pointcloud those points that we don't want to have
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*MyPointCloud).size(); i++)
  {
    pcl::PointXYZ pt(MyPointCloud->points[i].x, MyPointCloud->points[i].y, MyPointCloud->points[i].z);
    bool vehicle = pow(pt.x,2) + pow(pt.y,2) <= pow(ROVER_RADIUS,2);
    bool outOfBoundaries = (pt.x<SEARCH_AREA_LIMITS[0] || pt.x>SEARCH_AREA_LIMITS[1]) ||
                           (pt.y<SEARCH_AREA_LIMITS[2] || pt.y>SEARCH_AREA_LIMITS[3]) ||
                           (pt.z<SEARCH_AREA_LIMITS[4] || pt.z>SEARCH_AREA_LIMITS[5]);
    if (outOfBoundaries || vehicle) //remove these points
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(MyPointCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*MyPointCloud);
  std::cout << "Cuttering.. Number of Points now: " << MyPointCloud->size() << std::endl;

  //Code that implements the SMRF segmentation algorithm

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInitialDistance (0.5f);
  pmf.setInputCloud (MyPointCloud);
  pmf.setMaxWindowSize (MAXIMUM_WINDOW_SIZE);
  pmf.setSlope (SLOPE_THRESHOLD);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract2;
  extract2.setInputCloud (MyPointCloud);
  extract2.setIndices (ground);
  extract2.filter (*groundPointCloud);

  // Extract non-ground returns
  extract2.setNegative (true);
  extract2.filter (*obstaclesPointCloud);

  std::cout << "MorphologicalFilter.. " << std::endl;
  std::cout << "Number of Points considered Ground: " << groundPointCloud->size() << std::endl;
  std::cout << "Number of Points considered Obstacles: " << obstaclesPointCloud->size() << std::endl;
  std::cout << "Start of Clustering.." << std::endl;

  // Clustering
 
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (obstaclesPointCloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (MIN_DISTANCE_CLUSTERS);
  ec.setMinClusterSize (MIN_POINTS_CLUSTERS);
  ec.setMaxClusterSize (MAX_POINTS_CLUSTERS);
  ec.setSearchMethod (tree);
  ec.setInputCloud (obstaclesPointCloud);
  ec.extract (cluster_indices);

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*MyPointCloud, minPt, maxPt);
  int width = int((maxPt.x - minPt.x)/CELL_SIZE); 
  int height = int((maxPt.y - minPt.y)/CELL_SIZE); 

  nav_msgs::MapMetaData mapMeta;

  mapMeta.resolution = CELL_SIZE;
  mapMeta.width = width;
  mapMeta.height = height;

  geometry_msgs::Pose oPose;
  oPose.position.x = 0; 
  oPose.position.y = 0; 
  mapMeta.origin = oPose;

  nav_msgs::OccupancyGrid oMap;
  oMap.info = mapMeta;
  oMap.data.resize(width * height);
  oMap.header.frame_id = cloud_msg->header.frame_id;

  float closestPoint[3] = {50.0, 50.0, 50.0};
  
  switch(TYPE_OF_OCCUPANCY_GRID) {
    case 0:   //Case of small quadratic shape
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->push_back ((*obstaclesPointCloud)[*pit]); //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //Code for uploading the Occupancy Grid Message
      for (const auto& point: *cloud_cluster) {
        if (sqrt(pow(point.x,2)+pow(point.y,2)+pow(point.z,2))<sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))) {
          closestPoint[0] = point.x; closestPoint[1] = point.y; closestPoint[2] = point.z; 
          std::cout << "Distance closest Point: "
            << sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))
            << std::endl;
        }
        if (sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))<OBS_DIST_THRESHOLD) {
          std::cout << "TOO CLOSE TO AN OBSTACLE" << std::endl;
        }
        // check for nan points
        if (!(isnan(point.x) | isnan(point.y)))
        {
            int x = int ((point.x / CELL_SIZE) - (minPt.x / CELL_SIZE));
            int y = int ((point.y / CELL_SIZE) - (minPt.y / CELL_SIZE));
            if (x < width && y < height)
            {
                oMap.data[MAP_IDX(width, x, y)] = 100;
            }
        }
      } 
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;}
    break;
    case 1:   //Case of great quadratic shape
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->push_back ((*obstaclesPointCloud)[*pit]); //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //Code for uploading the Occupancy Grid Message
      for (int mapIndex = 0; mapIndex<width*height; mapIndex++) {
        for (const auto& point: *cloud_cluster) {
          if (sqrt(pow(point.x,2)+pow(point.y,2)+pow(point.z,2))<sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))) {
          closestPoint[0] = point.x; closestPoint[1] = point.y; closestPoint[2] = point.z; 
          std::cout << "Distance closest Point: "
            << sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))
            << std::endl;
        }
        if (sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))<OBS_DIST_THRESHOLD) {
          std::cout << "TOO CLOSE TO AN OBSTACLE" << std::endl;
        }
          int x = int ((point.x / CELL_SIZE) - (minPt.x / CELL_SIZE));
          int y = int ((point.y / CELL_SIZE) - (minPt.y / CELL_SIZE));
          if (x < width && y < height && (mapIndex == x+y))
            {
                oMap.data[MAP_IDX(width, x, y)] = 100;
                break;
            }
      }
      
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;}
    }
    break;
    case 2:   //Case of rectangular boundaries as footprint
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->push_back ((*obstaclesPointCloud)[*pit]); //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      float BoundariesOfObs[4] = {50.0, -50.0, 50.0, -50.0};

      //Code for uploading the Occupancy Grid Message
      for (const auto& point: *cloud_cluster) {
        if (sqrt(pow(point.x,2)+pow(point.y,2)+pow(point.z,2))<sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))) {
          closestPoint[0] = point.x; closestPoint[1] = point.y; closestPoint[2] = point.z; 
          std::cout << "Distance of closest Point: "
            << sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))
            << std::endl;
        }
        if (sqrt(pow(closestPoint[0],2)+pow(closestPoint[1],2)+pow(closestPoint[2],2))<OBS_DIST_THRESHOLD) std::cout << "TOO CLOSE TO AN OBSTACLE" << std::endl;
        if (point.x < BoundariesOfObs[0]) BoundariesOfObs[0] = point.x;
        else if (point.x > BoundariesOfObs[1]) BoundariesOfObs[1] = point.x;
        if (point.y < BoundariesOfObs[2]) BoundariesOfObs[2] = point.y;
        else if (point.y > BoundariesOfObs[3]) BoundariesOfObs[3] = point.y;
      }
      int xMinOCC = int((BoundariesOfObs[0]/CELL_SIZE) - (minPt.x/CELL_SIZE));
      int xMaxOCC = int((BoundariesOfObs[1]/CELL_SIZE) - (minPt.x/CELL_SIZE));
      int yMinOcc = int((BoundariesOfObs[2]/CELL_SIZE) - (minPt.y/CELL_SIZE));
      int yMaxOcc = int((BoundariesOfObs[3]/CELL_SIZE) - (minPt.y/CELL_SIZE));
      std::cout << xMinOCC << ":" << xMaxOCC << "|" << yMinOcc << ":" << yMaxOcc << std::endl;
      for (int x=xMinOCC; x<=xMaxOCC; x++){
        for (int y=yMinOcc; y<=yMaxOcc; y++) {
          if (x < width && y < height)
            { oMap.data[MAP_IDX(width, x, y)] = 100; }
        }}
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    } 
    break; 
  }
  
    // sistemare rettangoli, sistemare risoluz bassa (ottimizz) e salvare immagini 

  occupancy_pub.publish(oMap);
  ros::Time endTime = ros::Time::now(); //Get end time 
  std::cout << "Execution Time: " << (endTime-startTime).toSec() << std::endl;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;
}

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "vision_node");

  // Create a ROS node handle
  ros::NodeHandle n;
  ROS_INFO("Execution: Vision Node");
  ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("gps_data",1000,gps_callback);
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("point_cloud", 1000, segmentationOfEnvironment);
  occupancy_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1000);
  ros::spin();

  return 0;
}