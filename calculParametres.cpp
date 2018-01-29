// ********************************************************************************
// UTILISATION//
// avoir main.cpp CmakeLists.txt et pcl_visualizer_demo.cpp dans un dossier src 
// ouvrir un terminal dans le dossier parent de src et faire cmake src
// puis faire make (toujours depuis le même endroit)
// on obtient notre executable pcl_visualizer_demo
// ./pcl_visualizer_demo avec les argument que l'on désire
// ********************************************************************************
//			**************************************
//	 	 ******	en une fois: cmake src && make	********
//			**************************************

//librairies pour le calcul des eigenvalues et eigenvectors
//on utilisera la méthode de jacobi car on a des matrices symétriques
# include <cstdlib>
# include <iomanip>
# include <cmath>
# include <ctime>
# include <cstring>



# include "jacobi_eigenvalue.hpp"
# include "jacobi_eigenvalue.cpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/impl/point_types.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
//permet la lecture de fichiers au format .ply
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

// double[] vecteurs est le tableau des distances entre le point considéré et ses voisins, k est le nombre de voisins
void calculParametre( double vecteurs [][3] , int k )
{
	//calcul de la matrice de covarience des voisins: C 
	double matCov [3*3];
	//que l'on initialise à 0
	for( int i = 0 ; i < 9 ; i++ )
	{
		matCov[i] = 0.0;
	}
	//pour chaques voisins on calcul vecteur[i][k]*vecteur[i][k]^T ( ^T est la transposée)
	for( int i = 0 ; i < k ; i++ )
	{
		//on calcule la matrice de covarience
		matCov[0] = matCov[0] + vecteurs[i][0]*vecteurs[i][0];
		matCov[1] = matCov[1] + vecteurs[i][0]*vecteurs[i][1];
		matCov[2] = matCov[2] + vecteurs[i][0]*vecteurs[i][2];
		
		matCov[3] = matCov[3] + vecteurs[i][1]*vecteurs[i][0];
		matCov[4] = matCov[4] + vecteurs[i][1]*vecteurs[i][1];
		matCov[5] = matCov[5] + vecteurs[i][1]*vecteurs[i][2];
		
		matCov[6] = matCov[6] + vecteurs[i][2]*vecteurs[i][0];
		matCov[7] = matCov[7] + vecteurs[i][2]*vecteurs[i][1];
		matCov[8] = matCov[8] +vecteurs[i][2]*vecteurs[i][2];
	}
	/*double matCov [4*4] = {
      4.0,  -30.0,    60.0,   -35.0, 
    -30.0,  300.0,  -675.0,   420.0, 
     60.0, -675.0,  1620.0, -1050.0, 
    -35.0,  420.0, -1050.0,   700.0 };*/
	
	//ici on a donc notre matrice 3x3 de covarience. Reste à calculer ses valeurs propres grace a l'algorithme de jacobi
	double d [3]; // la ou l'on stocke nos eigenvalues
	double error_frobenius;
	int it_max;
	int it_num;
	int n = 3; //taille de notre matrice symétrique (ici 3x3)
	int rot_num;
	double v [3*3]; // la ou l'on stocke nos eigenvectors

	cout << "\n";
	cout << "TEST01\n";
	cout << "  For a symmetric matrix A,\n";
	cout << "  JACOBI_EIGENVALUE computes the eigenvalues D\n";
	cout << "  and eigenvectors V so that A * V = D * V.\n";

	r8mat_print ( n , n , matCov , "  Input matrix A:" );

	it_max = 100;

	jacobi_eigenvalue ( n, matCov, it_max, v, d, it_num, rot_num );

	cout << "\n";
	cout << "  Number of iterations = " << it_num << "\n";
	cout << "  Number of rotations  = " << rot_num << "\n";

	r8vec_print ( n, d, "  Eigenvalues D:" );

	r8mat_print ( n, n, v, "  Eigenvector matrix V:" );
	//  Compute eigentest.
	error_frobenius = r8mat_is_eigen_right ( n, n, matCov, v, d );
	cout << "\n";
	cout << "  Frobenius norm error in eigensystem A*V-D*V = " << error_frobenius << "\n";
	   
	return;
}

int main()
{
	double vec [1][3];
	vec[0][0] = 7.0;
	vec[0][1] = 32.5;
	vec[0][2] = 13.0;
	int nbVoisins = 1;
	calculParametre( vec , nbVoisins );
}
