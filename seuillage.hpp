#ifndef _SEUILLAGE_
#define _SEUILLAGE_

#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "seuillage.cpp"

std::vector<int> seuillage(double poids[], pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double seuil);

#endif
