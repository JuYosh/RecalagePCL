/* \author Geoffrey Biggs */


#include <iostream>
#include <vector>
#include <ctime>
#include <string>

#include <pcl/io/vtk_lib_io.h>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
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


void kppv(double X, double Y, double Z, int K, double coords[][3], pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud )
{
	//Création du kdtree et input du nuage
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pCloud);
	
	//Définition du point de recherche
	pcl::PointXYZ searchPoint;
	searchPoint.x = X;
	searchPoint.y = Y;
	searchPoint.z = Z;

	//Vecteurs d'ID et Distances poru la recherche
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
		}	
	}
}




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

	/*EXEMPLE D'UTILISATION DE KPPV*/
	int K = 2;
	double cc[K][3];

	//On passe les X, Y et Z du searchPoint, K, le tableau à remplir et le cloud
	kppv(0, 0, 0, K, cc, cloud);
	for(int i = 0; i < K; i++)
		cout << "Coordonnées du vecteur X Y Z " << i << " : " << cc[i][0] << " " << cc[i][1] << " " << cc[i][2] << endl;


	return EXIT_SUCCESS;
}
