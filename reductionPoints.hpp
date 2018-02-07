#ifndef _REDUCTIONPOINTS_
#define _REDUCTIONPOINTS_

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>

#include "reductionPoints.cpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr reductionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

#endif
