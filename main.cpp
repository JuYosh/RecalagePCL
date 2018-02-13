#include <time.h>
#include <vector>
#include <string>
#include <iterator>
#include <iostream>
#include <stdlib.h>

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "seuillage.hpp"
#include "creerNuage.hpp"
#include "correspondance.hpp"
#include "calculParametres.hpp"
#include "plusProchesVoisins.hpp"
#include "calculTransformations.hpp"
#include "matchingError.hpp"
#include "creerNuageSeuille.hpp"
#include "appliquerTransformations.hpp"

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
	// Pointeurs sur les nuages
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_S, cloud_M;
	std::string inputFilenameSource = "bunnyBack2.stl";
	std::string inputFilenameTarget = "bunnyHead2.stl";
	
	int K = 5; //nombre de voisins
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
	cloud_S = creerNuage(inputFilenameSource);
	cloud_M = creerNuage(inputFilenameTarget);

	//on reduit le nombre de points du nuage Source
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (cloud_S);
 	sor.setLeafSize (1.0f, 1.0f, 1.0f);
  	sor.filter (*cloud_S);

	//on reduit le nombre de points du nuage Target
  	sor.setInputCloud (cloud_M);
  	sor.filter (*cloud_M); 

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
	
	//premiere boucle de parcours des points du nuage SOURCE pour calculer le KPPV, puis tau et la normale
	//affichage de la "progression" du calcul
	/*cout <<		"ETAPE 1" << endl;
	cout << 	"Progression:	0	10	20	30	40	50	60	70	80	90	100	" << endl;
	cout << 	"			    [#"; //on ajoute un "#" tout les 10/4 = 2.5%*/
	tmp = (double)(nbPointsCloud_S);
	int compteur = 0;
	
	t0 = clock();
	for( int i = 0 ; i < nbPointsCloud_S ; i++ )
	{
		//On utilise KKPV en passant les X, Y et Z du searchPoint, K, le tableau à remplir et le cloud
		kppv( cloud_S->points[i].x , cloud_S->points[i].y , cloud_S->points[i].z , K , vecteurs , cloud_S );
		
		//on demande le calcul des parametres normale et tau
		calculParametre( vecteurs , K , tabNormal_S , tabTau_S , i );

	}
	
	
	// deuxieme boucle de parcours des points du nuage pour calculer le KPPV, et acceder au coef angulaire et au poids
	t0 = clock();
	double tmpNormale[3];
	for( int i = 0 ; i < nbPointsCloud_S ; i++ )
	{
		kppv2( cloud_S->points[i].x , cloud_S->points[i].y , cloud_S->points[i].z , K , vecteurs , cloud_S , tabNormal_S);
		
		for(int l = 0; l < 3; l++)
		{
			tmpNormale[l] = tabNormal_S[i][l];
			
		}
		calculeAngleTau( tabTau_S[i] , tmpNormale , vecteurs , K , tabAngle_S , tabPoids_S , i );
	}
	

	//premiere boucle de parcours des points du nuage TARGET pour calculer le KPPV, puis tau et la normale
	//affichage de la "progression" du calcul
	//cout << 	"Progression:	0	10	20	30	40	50	60	70	80	90	100		\n";
	//cout << 	"			    [#"; //on ajoute un "#" tout les 10/4 = 2.5%
	
	t0 = clock();
	for( int i = 0 ; i < nbPointsCloud_M ; i++ )
	{

		//On utilise KKPV en passant les X, Y et Z du searchPoint, K, le tableau à remplir et le cloud
		kppv( cloud_M->points[i].x , cloud_M->points[i].y , cloud_M->points[i].z , K , vecteurs , cloud_M );

		
		//on demande le calcul des parametres normale et tau
		calculParametre( vecteurs , K , tabNormal_M , tabTau_M , i );
		
	}
	tmp = (double)(nbPointsCloud_M);
	cout << endl << "Nombre de points du nuage TARGET: " << nbPointsCloud_M << endl;
	cout << "Nombre de points du nuage SOURCE: " << nbPointsCloud_S << endl;
	// deuxieme boucle de parcours des points du nuage pour calculer le KPPV, et acceder au coef angulaire et au poids
	t0 = clock();
	for( int i = 0 ; i < nbPointsCloud_M ; i++ )
	{
		kppv2( cloud_M->points[i].x , cloud_M->points[i].y , cloud_M->points[i].z , K , vecteurs , cloud_M , tabNormal_M);
		
		for(int l = 0; l < 3; l++)
		{
			tmpNormale[l] = tabNormal_M[i][l];
			
		}
		
		calculeAngleTau( tabTau_M[i] , tmpNormale , vecteurs , K , tabAngle_M , tabPoids_M , i );
	}
	
	//cout << "Pony" << endl;
	//indices_S = seuillage(tabPoids_S, cloud_S, 0);
	
	/*for(std::vector<int>::iterator i = indices_S.begin() ; i != indices_S.end() ; i++)
	{
		cloud_S->points[*i].r = 255;
		cloud_S->points[*i].g = 0;
		cloud_S->points[*i].b = 0;
	}*/
	//cout << "Bloap" << endl;
	//indices_M = seuillage(tabPoids_M, cloud_M, 0);
	
	//on fait un seuillage sur nos nuages de points
	std::vector<int> indiceSeuillageSource;
	std::vector<int> indiceSeuillageTarget;
	//initialisation des threshold pour le seuillage
	double seuil_S , seuil_M;
	seuil_S = 8.0;
	seuil_M = 8.0;
	// pour sample de 1.0
	// 19 -> seuilS/M 6/2 cor=0
	// 18 -> seuilS/M 28/16 cor=0
	// 17 -> seuilS/M 58/34 cor=0
	// 16 -> seuilS/M 152/198 cor=0
	
	double A [3] , B[3] , A_normal [3] , B_normal [3];
	double rotation [3][3];
	double translation [3];
	indiceSeuillageTarget = seuillage( tabPoids_M , cloud_M , seuil_M );
	indiceSeuillageSource = seuillage( tabPoids_S , cloud_S , seuil_S );
	
	/*cout << "**********************NUAGE SOURCE***********" << endl;
	for( int i = 0 ; i < indiceSeuillageSource.size() ; i++ )
	{
		cout << "ANGLE = " << tabAngle_S[i] << "	NORMALE = " << tabNormal_S[i][0] << " | " << tabNormal_S[i][1] << " | " << tabNormal_S[i][2] << " 	COURBURE = " << tabTau_S[i] << endl;
	}
	cout << "**********************NUAGE TARGET***********" << endl;
	for( int i = 0 ; i < indiceSeuillageTarget.size() ; i++ )
	{
		cout << "ANGLE = " << tabAngle_M[i] << "	NORMALE = " << tabNormal_M[i][0] << " | " << tabNormal_M[i][1] << " | " << tabNormal_M[i][2] << " 	COURBURE = " << tabTau_M[i]  << endl;
	}*/
	int im, is;
	double zMaxM = -1000.0 , zMaxS = -1000.0;
	for( int i = 0 ; i < nbPointsCloud_M; i++ )
	{
		if( cloud_M->points[i].z > zMaxM )
		{
			zMaxM = cloud_M->points[i].z;
			im = i;
		}
			
	}
	for( int i = 0 ; i < nbPointsCloud_S; i++ )
	{
		if( cloud_S->points[i].z > zMaxS )
		{
			zMaxS = cloud_S->points[i].z;
			is = i;
		}
			
	}
	
	
	

	cout << "Z_M = " << zMaxM << "		Z_S = " << zMaxS << endl;
	cout << "Nb pts seuilles S = " << indiceSeuillageSource.size() << endl;
	cout << "Nb pts seuilles M = " << indiceSeuillageTarget.size() << endl;


	cout << "M Normale  X : " << tabNormal_M[im][0] << endl;
	cout << "M Normale  Y : " << tabNormal_M[im][1] << endl;
	cout << "M Normale  Z : " << tabNormal_M[im][2] << endl;

	cout << "S Normale  X : " << tabNormal_S[is][0] << endl;
	cout << "S Normale  Y : " << tabNormal_S[is][1] << endl;
	cout << "S Normale  Z : " << tabNormal_S[is][2] << endl;

	cout << "Angle M : "<< tabAngle_M[im] << endl;
	cout << "Angle S : "<< tabAngle_S[is] << endl;


	//calcul des correspondances entres les deux nuages
	std::vector<std::vector<int> > correspondances = correspondance(tabAngle_S, tabAngle_M, tabTau_S, tabTau_M, 0.00001, 0.00001, indiceSeuillageSource, indiceSeuillageTarget);
	int indiceCor = 0;
	int tmpIndiceCor [2];
	
	//cout << "Nombre de correspondances comptes = " << nbCor << endl;
	cout << "Nombre de correspondances = " << correspondances.size() << endl;
	
	for( int i = 0 ; i < correspondances.size() ; i++ )
	{
		cloud_S->points[ correspondances[i][0] ].g = 0;
		cloud_S->points[ correspondances[i][0] ].b = 0;
		//cout << "ANGLE = " << tabAngle_S[ indiceSeuillageSource[i] ] << "	NORMALE = " << tabNormal_S[indiceSeuillageSource[i]][0] << " | " << tabNormal_S[indiceSeuillageSource[i]][1] << " | " << tabNormal_S[indiceSeuillageSource[i]][2] << " 	COURBURE = " << tabTau_S[indiceSeuillageSource[i]] << endl;
		
	}
	cout << endl << "******************************" << endl;
	for( int i = 0 ; i < correspondances.size(); i++ )
	{
		cloud_M->points[ correspondances[i][1] ].b = 0;
		cloud_M->points[ correspondances[i][1] ].r = 0;
		cloud_M->points[ correspondances[i][1] ].g = 0;
		//cout << "ANGLE = " << tabAngle_M[indiceSeuillageTarget[i]] << "	NORMALE = " << tabNormal_M[indiceSeuillageTarget[i]][0] << " | " << tabNormal_M[indiceSeuillageTarget[i]][1] << " | " << tabNormal_M[indiceSeuillageTarget[i]][2] << " 	COURBURE = " << tabTau_M[indiceSeuillageTarget[i]]  << endl;
	}
	
	
	
	srand( time(NULL) );
	for( int luna = 0 ; luna < 3 ; luna++ )
	{		
		indiceCor = rand()%( correspondances.size() );//on prend au hasard une correspondance depuis le tableau d'indice: indiceSeuillageTarget
		indiceCor = luna;
		tmpIndiceCor[0] = correspondances[indiceCor][0];//on recupere le rand ieme point de correspondance
		tmpIndiceCor[1] = correspondances[indiceCor][1];
		A[0] = cloud_S->points[ tmpIndiceCor[0] ].x;	A[1] = cloud_S->points[ tmpIndiceCor[0] ].y;	A[2] = cloud_S->points[ tmpIndiceCor[0] ].z;
		B[0] = cloud_M->points[ tmpIndiceCor[1] ].x;	B[1] = cloud_M->points[ tmpIndiceCor[1] ].y;	B[2] = cloud_M->points[ tmpIndiceCor[1] ].z;
		
		A_normal[0] = tabNormal_S[ tmpIndiceCor[0] ][0];	A_normal[1] = tabNormal_S[ tmpIndiceCor[0] ][1];	A_normal[2] = tabNormal_S[ tmpIndiceCor[0] ][2];
		B_normal[0] = tabNormal_M[ tmpIndiceCor[1] ][0];	B_normal[1] = tabNormal_M[ tmpIndiceCor[1] ][1];	B_normal[2] = tabNormal_M[ tmpIndiceCor[1] ][2];
		
		
		calculTransformation( A , B ,  A_normal , B_normal , rotation, translation );
		
		cout << "*** ROTATION A APPLIQUER: " << endl <<
		rotation[0][0] << " | " << rotation[0][1] << " | " << rotation[0][2] << endl <<
		rotation[1][0] << " | " << rotation[1][1] << " | " << rotation[1][2] << endl <<
		rotation[2][0] << " | " << rotation[2][1] << " | " << rotation[2][2] << endl;
		cout << endl << "*** TRANSLATION A APPLIQUER: " << endl <<
		translation[0] << " | " << translation[1] << " | " << translation[2] << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud_M_Seuille;
		pCloud_M_Seuille = creerNuageSeuille( cloud_M , indiceSeuillageTarget );
		//pcl::transformPointCloud( *cloud_S, *cloud_S, transform);
		appliquerTransformations( cloud_S , rotation , translation );
		
		
		double mError = 0.0;
		//cout << "on est la 3" << endl;
		mError = matchingError( cloud_S , indiceSeuillageSource , pCloud_M_Seuille , tabAngle_S , tabTau_S , tabAngle_M , tabTau_M , correspondances );
		cout << endl << "matchin error = " << mError << endl << endl << endl;
	}	
	
	// Visualize PCL
	// Création de l'objet viewer avec un joli titre pour la fenêtre de visualisation
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Petite pony adorée"));
	// Définition de la couleur du fond
	viewer->setBackgroundColor(0.4, 0, 0.6);
	// Ajout des point du cloud au viewer, avec un ID string qui peut être utilisé pour identifier le cloud
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud_S);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud_M);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_S, rgb1, "sample cloud1"); 
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_M, rgb2, "sample cloud2"); 
	
	// Change la taille des points du nuage
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
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

