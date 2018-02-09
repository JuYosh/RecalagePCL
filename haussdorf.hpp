#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

double haussdorf(pcl::PointCloud<pcl::PointXYZ>::Ptr c1, pcl::PointCloud<pcl::PointXYZ>::Ptr c2);
