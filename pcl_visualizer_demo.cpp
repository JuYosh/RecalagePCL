/* \author Geoffrey Biggs */


#include <iostream>
#include <vector>
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

#include "calculParametres.hpp"
#include <time.h> 


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




// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
	// Lecture et affichage d'un nuage de point (fichier stl pour le moment)
	// Nom du fichier, cloud, pointeur sur cloud et caméra pour le viewer
	std::string inputFilename = "../Nuages/Essais_7.2.stl";
	pcl::PointCloud<pcl::PointXYZRGB> petitCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	std::vector<pcl::visualization::Camera> cam;

	// Permet d'ouvrir un fichier, et de le lire
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	reader->SetFileName(inputFilename.c_str());
	reader->Update();

	// Conversion de l'objet VTK en objet PCL
	pcl::io::vtkPolyDataToPointCloud(polydata, petitCloud);
	cloud = petitCloud.makeShared();

	//On initialise tous les points à blanc
	int nbPointsCloud = cloud->points.size();
	for(int i = 0; i < nbPointsCloud; i++)
	{
		cloud->points[i].r = 255;
		cloud->points[i].g = 255;
		cloud->points[i].b = 255;
	}


	/* Partie de code de Mathieu */	
	double * tabTau = (double*)(malloc( nbPointsCloud * sizeof(*tabTau) ) ); //tableau ou l'on stocke le tau de chaque points
	if(tabTau == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabTau!\nFin du programme.";
		return EXIT_FAILURE;
	}
	
	double **tabNormal;				//tableau ou l'on stocke la normale de chaque points
	//allocation de la premiere dimension
	tabNormal = (double**)( malloc( nbPointsCloud * sizeof(*tabNormal)) );       //On alloue 'taille1' pointeurs.
	if(tabNormal == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabNormal!\nFin du programme.";
		return EXIT_FAILURE;
	}
	//allocation de la deuxieme dimension
	for( int i = 0 ; i < nbPointsCloud ; i++ )
	{
		tabNormal[i] = (double*)( malloc( 3 * sizeof(*(tabNormal[i]))) );   //On alloue des tableaux de 'taille2' variables.
		if(tabNormal[i] == NULL)
		{
			cout << "Erreur lors de l'assignation memoire detabNormal dans la deuxieme dimention!\nFin du programme.";
			return EXIT_FAILURE;
		}
	}
	
	
	double *tabAngle = (double*)(malloc( nbPointsCloud * sizeof(*tabAngle) ) ); //tableau ou l'on stocke l'angle normale de chaque points
	if(tabAngle == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabAngle!\nFin du programme.";
		return EXIT_FAILURE;
	}
	
	double *tabPoids = (double*)(malloc( nbPointsCloud * sizeof(*tabPoids) ) ); //tableau ou l'on stocke le poids de chaque points
	if(tabPoids == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabPoids!\nFin du programme.";
		return EXIT_FAILURE;
	}

	int K = 3; //nombre de voisins
	double vecteurs[K][3]; //vecteur des distance aux voisins

	//premiere boucle de parcours des points du nuage pour calculer le KPPV, puis tau et la normale
	//affichage de la "progression" du calcul
	//cout << 	"Progression:	0	10	20	30	40	50	60	70	80	90	100		\n";
	//cout << 	"			    [#"; //on ajoute un "#" tout les 10/4 = 2.5%
	double progression = 0.0;
	double tmp;
	clock_t t0 , t1 , t2;
	t0 = clock();
	for( int i = 0 ; i < 10 ; i++ )
	{
		/*//affichage de la barre de progression
		if( (double)(i) > progression*0.025*nbPointsCloud )
		{
			cout <<  "#";
			progression = progression + 1.0;
		}*/

		//On utilise KKPV en passant les X, Y et Z du searchPoint, K, le tableau à remplir et le cloud
		kppv( cloud->points[i].x , cloud->points[i].y , cloud->points[i].z , K , vecteurs , cloud );
		
		//on demande le calcul des parametres normale et tau
		calculParametre( vecteurs , K , tabNormal , tabTau , i );
		if( i == 0 )
		{
			t1 = clock();
			tmp = (double)(t1-t0)/CLOCKS_PER_SEC;
			cout << "Execution d'une fois " << K << "ppv et calcul parametre = " << tmp << " s\n" ;
		}
			
		if( i == 9 )
		{
			t2 = clock();
			tmp = (double)(t2-t0)/CLOCKS_PER_SEC;
			cout << "Execution de 10 fois " << K << "ppv et calcul parametre = " << tmp << " s\n" ;
		}
	}
	tmp = (double)(nbPointsCloud);
	cout << "Nombre de points du nuage: " << tmp << "\n";
	
	//deuxieme boucle de parcours des points du nuage pour calculer le KPPV, et acceder au coef angulaire et au poids
	t0 = clock();
	for( int i = 0 ; i < 10 ; i++ )
	{
		kppv( cloud->points[i].x , cloud->points[i].y , cloud->points[i].z , K , vecteurs , cloud );
		calculeAngleTau( tabTau[i] , tabNormal[i] , vecteurs , K , tabAngle , tabPoids , i );
		if( i == 0 )
		{
			t1 = clock();
			tmp = (double)(t1-t0)/CLOCKS_PER_SEC;
			cout << "Excecution d'une fois " << K << "ppv et calcul angle/poids = " << tmp << " s\n" ;
		}
			
		if( i == 9 )
		{
			t2 = clock();
			tmp = (double)(t2-t0)/CLOCKS_PER_SEC;
			cout << "Excecution de 10 fois " << K << "ppv et calcul angle/poids = " << tmp << " s\n" ;
		}
	}


	// Visualize PCL
		// Création de l'objet viewer avec un joli titre pour la fenêtre de visualisation
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Petite pièce adorée"));
		// Définition de la couleur du fond
	viewer->setBackgroundColor(0.4, 0, 0.6);
		// Ajout des point du cloud au viewer, avec un ID string qui peut être utilisé pour identifier le cloud
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud"); 
// Change la taille des points du nuage
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
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


	return EXIT_SUCCESS;
}
