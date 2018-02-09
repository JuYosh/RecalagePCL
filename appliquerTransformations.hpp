#ifndef _APPLIQUER_TRANSFORMATIONS
#define _APPLIQUER_TRANSFORMATIONS
#include "appliquerTransformations.cpp"
extern void appliquerTransformations( pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudSource , double rotation [3][3], double translation [3] );
#endif
