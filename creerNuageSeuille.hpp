#ifndef _CREERNUAGESEUILLE_
#define _CREERNUAGESEUILLE_

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include "creerNuageSeuille.cpp"

extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr creerNuageSeuille(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices);

#endif
