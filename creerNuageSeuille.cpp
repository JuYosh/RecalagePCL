#include "creerNuage.hpp"
#include "creerNuageSeuille.hpp"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr creerNuageSeuille(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices)
{
	std::vector<int>::iterator i;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr petitCloud (new pcl::PointCloud<pcl::PointXYZRGB>());

	for(i = indices.begin() ; i < indices.end() ; i++)
	{
		petitCloud->points.push_back(cloud->points[*i]);
	}
	
	return petitCloud;
}
