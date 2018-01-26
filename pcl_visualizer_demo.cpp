/* \author Geoffrey Biggs */


#include <iostream>
#include <vector>
#include <ctime>
#include <string>

#include <pcl/io/vtk_lib_io.h>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>



// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
	// Lecture et affichage d'un nuage de point (fichier stl pour le moment)
	// Nom du fichier, cloud, pointeur sur cloud et caméra pour le viewer
	std::string inputFilename = "../Nuages/Essais_7.2.stl";
	pcl::PointCloud<pcl::PointXYZ> petitCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<pcl::visualization::Camera> cam;

	// Permet d'ouvrir un fichier, et de le lire
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	reader->SetFileName(inputFilename.c_str());
	reader->Update();

	// Conversion de l'objet VTK en objet PCL
	pcl::io::vtkPolyDataToPointCloud(polydata, petitCloud);
	cloud = petitCloud.makeShared();
	
	// Visualize PCL
		// Création de l'objet viewer avec un joli titre pour la fenêtre de visualisation
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Petite pièce adorée"));
		// Définition de la couleur du fond
	viewer->setBackgroundColor(0.4, 0, 0.6);
		// Ajout des point du cloud au viewer, avec un ID string qui peut être utilisé pour identifier le cloud
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
		// Change la taille des points du nuage
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		// Ajout des axes (de taille 1.0)
	viewer->addCoordinateSystem(1.0);
		// Initialisation de la caméra
	viewer->initCameraParameters();
	viewer->getCameras(cam);
	cam[0].pos[0] = -300;
	cam[0].pos[1] = 10;
	viewer->setCameraPosition(cam[0].pos[0], cam[0].pos[1], cam[0].pos[2], cam[0].view[0], cam[0].view[1], cam[0].view[2], cam[0].focal[0], cam[0].focal[1], cam[0].focal[2]);
		// Boucle pour la visualisation
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	// Visualize VTK
	/*vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderer->AddActor(actor);
	renderer->SetBackground(.3, .6, .3); // Background color green

	renderWindow->Render();
	renderWindowInteractor->Start();*/
	
	
	
	
	// Recherche des K plus proches voisins d'un point (au hasard pour l'instant)
	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	
	//kdtree.setInputCloud(cloud);
	/*pcl::PointXYZ searchPoint;
	int K = 10;
	
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	cout << "K plus proches voisins de (" << searchPoint.x << ", "
		 << searchPoint.y << ", " << searchPoint.z << ") avec K = "
		 << K << endl;*/
	/*
	if(kdtree.nearestKSearch(serchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for(size_t i = 0 ; i < pointIdxNKNSearch.size() ; i++)
		{*/
			//cout << cloud.points[0].x << endl;
		//}
	//}

	return EXIT_SUCCESS;
}
