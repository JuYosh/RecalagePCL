#include "seuillage.hpp"

std::vector<int> seuillage(double *poids, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double seuil)
{
	std::vector<int> indices;

	for(size_t i = 0 ; i < cloud->points.size() ; i++)
	{
		if(poids[i] > seuil)
		{
			//cout << seuil << "	" << poids[i] << "	" << i << endl;
			indices.push_back(i);
		}
	}
	
	return indices;
}
