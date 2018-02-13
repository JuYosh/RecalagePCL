#ifndef _APPLIQUER_TRANSFORMATIONS
#define _APPLIQUER_TRANSFORMATIONS
#include "appliquerTransformations.cpp"
extern void appliquerRotation( pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudSource , double rotation [3][3]);
extern void appliquerTranslation( pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudSource , double translation [3]);
#endif
