#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

void visualize_normals(double** normals,  boost::shared_ptr<pcl::visualization::PCLVisualizer> view, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, int idMax, int pas);

