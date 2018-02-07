#ifndef _MATCHING_ERROR
#define _MATCHING_ERROR
#include "matchingError.cpp"
extern double norme2( double A [3] , double B [3] );
double matchingError( pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudSource , std::vector<int> indicePtsInteretSource , pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudTarget ,
						double tabAngleSource [] , double tabTauSource [] , double tabAngleTarget [] , double tabTauTarget [] , std::vector<std::vector<int> > vectCorresp );
