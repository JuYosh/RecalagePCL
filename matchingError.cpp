# include <cmath>
# include <vector> 
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
using namespace std;

//calcul de la distance en norme 2 (euclidienne) entre le point A et B de R3
double norme2( double A [3] , double B [3] )
{
	double res = 0.0;
	res = sqrt( pow( (A[0]-B[0]) , 2 ) + pow( (A[1]-B[1]) , 2 ) + pow( (A[2]-B[2]) , 2 ) );
	return res;
}

//fonction de calcul de la matching error, qui est fonction entre les points d'interets du nuage source et leurs 1-plus proche voisins respectifs (et leurs ecarts parametriques aussi)
double matchingError( pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudSource , std::vector<int> indicePtsInteretSource , pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudTarget ,
						double tabAngleSource [] , double tabTauSource [] , double tabAngleTarget [] , double tabTauTarget [] , std::vector<std::vector<int> > vectCorresp )
{
	int nbPointsInteretSource;
	nbPointsInteretSource = sizeof( indicePtsInteretSource );
	
	double matchingError = 0.0;
	//pour chaque points d'interet on calcule la matching error
	for( int i = 0 ; i < nbPointsInteretSource , i++ )
	{
		//test si le point de S(source) à un point de correspondance, dans ce cas on calcule l'erreur
		if( vectCorresp[ indicePtsInteretSource[i] ][1] != -1 )
		{
			//Création du kdtree et input du nuage target
			pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
			kdtree.setInputCloud(pCloudTarget);
			
			//Définition du point de recherche: le poitn d'interet i du nuage source
			pcl::PointXYZRGB searchPoint;
			searchPoint.x = pCloudSource->points[ indicePtsInteretSource[i] ].x;
			searchPoint.y = pCloudSource->points[ indicePtsInteretSource[i] ].y;
			searchPoint.z = pCloudSource->points[ indicePtsInteretSource[i] ].z;

			//Vecteurs d'ID et Distances pour la recherche
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);

			
			//Recherche par rapport au searchPoint et remplissage du tableau en paramètre
			if ( kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				pCloudTarget->points[ pointIdxNKNSearch[0] ].x;
				pCloudTarget->points[ pointIdxNKNSearch[0] ].y;
				pCloudTarget->points[ pointIdxNKNSearch[0] ].z;
				//on ajoute la distance entre nos 2 points de correspondance à l'erreur
				matchingError = matchingError + norme2( pCloudTarget->points[ pointIdxNKNSearch[0] ] , pCloudSource->points[ indicePtsInteretSource[i] ] );
				//on ajoute la difference des parametres angulaires entre ces deux points
				matchingError = matchingError + fabs( tabAngleTarget[ pointIdxNKNSearch[0] ] - tabAngleSource[ indicePtsInteretSource[i] ] );
				//on ajoute la difference des parametres tau (courbure) entre ces deux points
				matchingError = matchingError + fabs( tabTauTarget[ pointIdxNKNSearch[0] ] - tabTauSource[ indicePtsInteretSource[i] ] );
			}
		}
	}
	return matchingError;
}
