#include <iostream>
#include <pcl/point_cloud.h>


//Renvoit la distance de Haussdorf normalisée entre 2 nuages de points (Pour chaque point, distance du point le plus proche de l'autre nuage)
double haussdorf(pcl::PointCloud<pcl::PointXYZ>::Ptr c1, pcl::PointCloud<pcl::PointXYZ>::Ptr c2)
{
	double sum = 0;
	int s = c1->points.size();

	//Création du kdtree et input du nuage
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(c2);

	pcl::PointXYZ searchPoint;

	std::vector<int> pointIdxNKNSearch(2);
	std::vector<float> pointNKNSquaredDistance(2);	


	for(int i = 0; i < s; i++)
	{
		searchPoint.x = c1->points[i].x;
		searchPoint.y = c1->points[i].y;
		searchPoint.z = c1->points[i].z;
		
		if ( kdtree.nearestKSearch (searchPoint, 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
			//Fonction à surveiller, est ce que le plus proche voisin du point est le point lui même ? si c'est le cas, on prend l'indice 1 et non 0
			sum += pointNKNSquaredDistance[1];
		}
	}                        
	return sum/s;
}
