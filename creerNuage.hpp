#ifndef _CREERNUAGE_
#define _CREERNUAGE_

#include <string>
#include <iostream>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>

#include "creerNuage.cpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr creerNuage(const std::string inputFilename);

#endif
