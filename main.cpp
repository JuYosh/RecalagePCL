#include <time.h>
#include <vector>
#include <string>
#include <iterator>
#include <iostream>

#include <pcl/io/vtk_lib_io.h>
#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkPLYReader.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>

#include "seuillage.hpp"
#include "creerNuage.hpp"
#include "correspondance.hpp"
#include "calculParametres.hpp"
#include "plusProchesVoisins.hpp"


// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
	// Pointeurs sur les nuages
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_S, cloud_M;
	std::string inputFilename = "../Nuages/Essais_7.1.stl";
	
	int K = 10; //nombre de voisins
	int nbPointsCloud_S, nbPointsCloud_M; // nombre de points dans le nuage
	double vecteurs[K][3]; // vecteur des distance aux voisins
	
	double * tabTau_S; // tableau ou l'on stocke la courbure de chaque points
	double **tabNormal_S; // tableau ou l'on stocke la normale de chaque points
	double *tabAngle_S; // tableau ou l'on stocke l'angle normale de chaque points
	double *tabPoids_S; // tableau ou l'on stocke le poids de chaque points
	double * tabTau_M;
	double **tabNormal_M;
	double *tabAngle_M;
	double *tabPoids_M;
	
	// Pour les calculs de progression de l'algo
	double progression = 0.0;
	double tmp;
	clock_t t0 , t1 , t2;
	
	// Pour le seuillage et la mise en correspondance des points
	std::vector<int> indices_S, indices_M;
	
	// Caméra pour le viewer
	std::vector<pcl::visualization::Camera> cam;
	
	// Ouverture du fichier
	cloud_S = creerNuage(inputFilename);
	cloud_M = creerNuage(inputFilename);

	//On initialise tous les points à blanc
	nbPointsCloud_S = cloud_S->points.size();
	nbPointsCloud_M = cloud_M->points.size();
	for(int i = 0; i < nbPointsCloud_S; i++)
	{
		cloud_S->points[i].r = 255;
		cloud_S->points[i].g = 255;
		cloud_S->points[i].b = 255;
	}
	for(int i = 0; i < nbPointsCloud_M; i++)
	{
		cloud_M->points[i].r = 255;
		cloud_M->points[i].g = 255;
		cloud_M->points[i].b = 255;
	}



	/* Partie de code de Mathieu */	
	tabTau_S = (double*)(malloc( nbPointsCloud_S * sizeof(*tabTau_S) ) );
	if(tabTau_S == NULL)
	{
		cout << "Erreur lors de l'assignation mémoire de tabTau_S !\nFin du programme.";
		return EXIT_FAILURE;
	}
	
	
	// allocation de la premiere dimension
	tabNormal_S = (double**)( malloc( nbPointsCloud_S * sizeof(*tabNormal_S)) );       //On alloue 'taille1' pointeurs.
	if(tabNormal_S == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabNormal_S !\nFin du programme.";
		return EXIT_FAILURE;
	}
	// allocation de la deuxieme dimension
	for( int i = 0 ; i < nbPointsCloud_S ; i++ )
	{
		tabNormal_S[i] = (double*)( malloc( 3 * sizeof(*(tabNormal_S[i]))) );   //On alloue des tableaux de 'taille2' variables.
		if(tabNormal_S[i] == NULL)
		{
			cout << "Erreur lors de l'assignation memoire de tabNormal_S dans la deuxieme dimention !\nFin du programme.";
			return EXIT_FAILURE;
		}
	}
	
	tabAngle_S = (double*)(malloc( nbPointsCloud_S * sizeof(*tabAngle_S) ) ); 
	if(tabAngle_S == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabAngle !\nFin du programme.";
		return EXIT_FAILURE;
	}
	
	tabPoids_S = (double*)(malloc( nbPointsCloud_S * sizeof(*tabPoids_S) ) );
	if(tabPoids_S == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabPoids !\nFin du programme.";
		return EXIT_FAILURE;
	}
	

	tabTau_M = (double*)(malloc( nbPointsCloud_M * sizeof(*tabTau_M) ) );
	if(tabTau_M == NULL)
	{
		cout << "Erreur lors de l'assignation mémoire de tabTau_S !\nFin du programme.";
		return EXIT_FAILURE;
	}
	
	// allocation de la premiere dimension
	tabNormal_M = (double**)( malloc( nbPointsCloud_M * sizeof(*tabNormal_M)) );       //On alloue 'taille1' pointeurs.
	if(tabNormal_M == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabNormal_S !\nFin du programme.";
		return EXIT_FAILURE;
	}
	// allocation de la deuxieme dimension
	for( int i = 0 ; i < nbPointsCloud_M ; i++ )
	{
		tabNormal_M[i] = (double*)( malloc( 3 * sizeof(*(tabNormal_M[i]))) );   //On alloue des tableaux de 'taille2' variables.
		if(tabNormal_M[i] == NULL)
		{
			cout << "Erreur lors de l'assignation memoire de tabNormal_S dans la deuxieme dimention !\nFin du programme.";
			return EXIT_FAILURE;
		}
	}
	
	tabAngle_M = (double*)(malloc( nbPointsCloud_M * sizeof(*tabAngle_M) ) ); 
	if(tabAngle_M == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabAngle !\nFin du programme.";
		return EXIT_FAILURE;
	}
	
	tabPoids_M = (double*)(malloc( nbPointsCloud_M * sizeof(*tabPoids_M) ) );
	if(tabPoids_M == NULL)
	{
		cout << "Erreur lors de l'assignation memoire de tabPoids !\nFin du programme.";
		return EXIT_FAILURE;
	}

	//premiere boucle de parcours des points du nuage pour calculer le KPPV, puis tau et la normale
	//affichage de la "progression" du calcul
	//cout << 	"Progression:	0	10	20	30	40	50	60	70	80	90	100		\n";
	//cout << 	"			    [#"; //on ajoute un "#" tout les 10/4 = 2.5%
	
	t0 = clock();
	for( int i = 0 ; i < 10 ; i++ )
	{
		//On utilise KKPV en passant les X, Y et Z du searchPoint, K, le tableau à remplir et le cloud
		kppv( cloud_S->points[i].x , cloud_S->points[i].y , cloud_S->points[i].z , K , vecteurs , cloud_S );
		
		//on demande le calcul des parametres normale et tau
		calculParametres( vecteurs , K , tabNormal_S , tabTau_S , i );
		/*if( i == 0 )
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
		}*/
	}
	tmp = (double)(nbPointsCloud_S);
	cout << "Nombre de points du nuage: " << tmp << "\n";
	
	// deuxieme boucle de parcours des points du nuage pour calculer le KPPV, et acceder au coef angulaire et au poids
	t0 = clock();
	for( int i = 0 ; i < 10 ; i++ )
	{
		kppv( cloud_S->points[i].x , cloud_S->points[i].y , cloud_S->points[i].z , K , vecteurs , cloud_S );
		calculeAngleTau( tabTau_S[i] , tabNormal_S[i] , vecteurs , K , tabAngle_S , tabPoids_S , i );
		/*if( i == 0 )
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
		}*/
	}
	

	//premiere boucle de parcours des points du nuage pour calculer le KPPV, puis tau et la normale
	//affichage de la "progression" du calcul
	//cout << 	"Progression:	0	10	20	30	40	50	60	70	80	90	100		\n";
	//cout << 	"			    [#"; //on ajoute un "#" tout les 10/4 = 2.5%
	
	t0 = clock();
	for( int i = 0 ; i < 10 ; i++ )
	{

		//On utilise KKPV en passant les X, Y et Z du searchPoint, K, le tableau à remplir et le cloud
		kppv( cloud_M->points[i].x , cloud_M->points[i].y , cloud_M->points[i].z , K , vecteurs , cloud_M );
		
		//on demande le calcul des parametres normale et tau
		calculParametres( vecteurs , K , tabNormal_M , tabTau_M , i );
		/*if( i == 0 )
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
		}*/
	}
	tmp = (double)(nbPointsCloud_M);
	cout << "Nombre de points du nuage: " << tmp << "\n";

	// deuxieme boucle de parcours des points du nuage pour calculer le KPPV, et acceder au coef angulaire et au poids
	t0 = clock();
	for( int i = 0 ; i < 10 ; i++ )
	{
		kppv( cloud_M->points[i].x , cloud_M->points[i].y , cloud_M->points[i].z , K , vecteurs , cloud_M );
		calculeAngleTau( tabTau_M[i] , tabNormal_M[i] , vecteurs , K , tabAngle_M , tabPoids_M , i );
		/*if( i == 0 )
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
		}*/
	}
	
	cout << "Bloap" << endl;
	indices_S = seuillage(tabPoids_S, cloud_S, 0);
	
	for(std::vector<int>::iterator i = indices_S.begin() ; i != indices_S.end() ; i++)
	{
		cloud_S->points[*i].r = 255;
		cloud_S->points[*i].g = 0;
		cloud_S->points[*i].b = 0;
	}
	cout << "Bloap" << endl;
	indices_M = seuillage(tabPoids_M, cloud_M, 0);
	
	for(std::vector<int>::iterator i = indices_M.begin() ; i != indices_M.end() ; i++)
	{
		cloud_M->points[*i].r = 255;
		cloud_M->points[*i].g = 0;
		cloud_M->points[*i].b = 0;
	}
	
	std::vector<std::vector<int> > correspondances = correspondance(tabAngle_S, tabAngle_M, tabTau_S, tabTau_M, 0.00001, 0.00001, indices_S, indices_M);
	
	for(int i = 0 ; i < correspondances.size() ; i++)
	{
		for(int j = 0 ; j < correspondances[i].size() ; j++)
		{
			cout << correspondances[i][j] << " ";
		}
		cout << endl;
	}

	// Visualize PCL
	// Création de l'objet viewer avec un joli titre pour la fenêtre de visualisation
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Petite pièce adorée"));
	// Définition de la couleur du fond
	viewer->setBackgroundColor(0.4, 0, 0.6);
	// Ajout des point du cloud au viewer, avec un ID string qui peut être utilisé pour identifier le cloud
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_S);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_S, rgb, "sample cloud"); 
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

