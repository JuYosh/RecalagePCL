#ifndef _PLUSPROCHESVOISINS_
#define _PLUSPROCHESVOISINS_

#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>

#include "plusProchesVoisins.cpp"

void kppv(double X, double Y, double Z, int K, double coords[][3], pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud );

#endif
