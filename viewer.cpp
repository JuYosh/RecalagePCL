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
void viewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloupy)
{
	std::vector<pcl::visualization::Camera> cam;

	// Visualize PCL
		// Création de l'objet viewer avec un joli titre pour la fenêtre de visualisation
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Petite pièce adorée"));
		// Définition de la couleur du fond
	viewer->setBackgroundColor(0.4, 0, 0.6);
		// Ajout des point du cloud au viewer, avec un ID string qui peut être utilisé pour identifier le cloud
	viewer->addPointCloud<pcl::PointXYZRGB>(cloupy, "sample cloud");
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
	}
}
