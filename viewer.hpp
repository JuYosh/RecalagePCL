#ifndef _VIEWER_
#define _VIEWER_

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

void viewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloupy);

#endif
