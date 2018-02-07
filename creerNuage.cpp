#include "creerNuage.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr creerNuage(const std::string inputFilename)
{
	pcl::PointCloud<pcl::PointXYZRGB> petitCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	// Lecture et affichage d'un nuage de point (fichier stl pour le moment)
	// Permet d'ouvrir un fichier, et de le lire
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	reader->SetFileName(inputFilename.c_str());
	reader->Update();

	// Conversion de l'objet VTK en objet PCL
	pcl::io::vtkPolyDataToPointCloud(polydata, petitCloud);
	cloud = petitCloud.makeShared();
	
	return cloud;
}
