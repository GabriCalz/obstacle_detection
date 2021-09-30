#include <ros/ros.h>

//Include for reading Point Cloud data from PCD files
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//Include for Visualization
#include <iostream>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//Include for Voxel Grid Filtering
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//Include for Cuttering 
#include <pcl/filters/extract_indices.h>

//Include for Morphological Filter Implementation
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

//Include for Clustering 
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//Create an OccupancyGrid
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
#include <nav_msgs/MapMetaData.h>

//Others 
#include <math.h>
#include <ros/ros.h> 
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
