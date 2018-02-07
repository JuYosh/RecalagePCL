#include "plusProchesVoisins.hpp"

void kppv(double X, double Y, double Z, int K, double coords[][3], pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud )
{
	//Création du kdtree et input du nuage
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(pCloud);
	
	//Définition du point de recherche
	pcl::PointXYZRGB searchPoint;
	searchPoint.x = X;
	searchPoint.y = Y;
	searchPoint.z = Z;

	//Vecteurs d'ID et Distances pour la recherche
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	
	//Recherche par rapport au searchPoint et remplissage du tableau en paramètre
	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
    		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		{				
			coords[i][0] = pCloud->points[ pointIdxNKNSearch[i] ].x - X;
			coords[i][1] = pCloud->points[ pointIdxNKNSearch[i] ].y - Y;
			coords[i][2] = pCloud->points[ pointIdxNKNSearch[i] ].z - Z;

			
			//On les met en rouge
			pCloud->points[ pointIdxNKNSearch[i] ].r = 255;
			pCloud->points[ pointIdxNKNSearch[i] ].g = 0;
			pCloud->points[ pointIdxNKNSearch[i] ].b = 0;
		}	
	}
}
