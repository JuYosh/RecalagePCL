#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

void appliquerTransformations( pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudSource , double rotation [3][3], double translation [3] )
{
	int nbPts;
	nbPts = pCloudSource->points.size();
	double tmp[3] , tmp1[3];
	//on applique la transformation a tout les points du nage source
	for( int i = 0 ; i < nbPts ; i++ )
	{
		
		//on applique la translation dans un premier temps
		pCloudSource->points[i].x = pCloudSource->points[i].x + translation[0];
		pCloudSource->points[i].y = pCloudSource->points[i].y + translation[1];
		pCloudSource->points[i].z = pCloudSource->points[i].z + translation[2];
		
		//puis la rotation
		tmp[0] = pCloudSource->points[i].x;
		tmp[1] = pCloudSource->points[i].y;
		tmp[2] = pCloudSource->points[i].z;
		setDbl3( tmp1 , tmp );
		produitMatVect( tmp1 , rotation , tmp );
		pCloudSource->points[i].x = tmp1[0];
		pCloudSource->points[i].y = tmp1[1];
		pCloudSource->points[i].z = tmp1[2];
	}
}
