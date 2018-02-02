#ifndef _CORRESPONDANCE_
#define _CORRESPONDANCE_

#include <cmath>
#include <vector>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "correspondance.cpp"

std::vector<std::vector<int> > correspondance(double angleS[], double angleM[], double courbureS[], double courbureM[], double seuilCourbure, double seuilAngle, std::vector<int> indicesS, std::vector<int> indicesM);

#endif
