#include "reductionPoints.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr reductionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	int num_output_points;
	float decimate_percent = 1.0/100.0;
	pcl::RandomSample<pcl::PointXYZRGB> random_sampler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
	
	num_output_points = (int) (decimate_percent * cloud->points.size());

	random_sampler.setInputCloud(cloud);
	random_sampler.setSample(num_output_points);
	random_sampler.filter(*cloud_filtered);

	cout << cloud->points.size() << endl;
	cout << cloud_filtered->points.size() << endl;
	
	return cloud_filtered;
}

// MAIN POUR LES TEST
/* 
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	std::string inputFilename = "../Nuages/Essais_7.1.stl";
	
	cloud = creerNuage(inputFilename);
	
	reductionPoints(cloud);
	
	return 0;
}
*/
