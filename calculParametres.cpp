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
# include <math.h>

#include <functional>   // std::minus
//#include <numeric>      // std::accumulate
//#include <algorithm>    // std::min_element, std::max_element

#include "jacobi_eigenvalue.hpp"
#include "jacobi_eigenvalue.cpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/impl/point_types.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include <pcl/console/parse.h>

using namespace std;

// double vecteurs[][3] est le tableau des distances vectorielles entre le point considéré et ses voisins, k est le nombre de voisins
// double tabNormal [][3] est le tableau des normales de chaqu'une des points du nuage, a remplir à la case [indiceTab]
//  double tabTau [] est le tableau des valeurs de tau de chaqu'une des points du nuage, a remplir à la case [indiceTab]
void calculParametre( double vecteurs [][3] , int k , double** tabNormal , double tabTau [] , int indiceTab  )
{
	//calcul de la matrice de covarience des voisins: C 
	double matCov [3*3];
	//approximation de la normale par le vecteur propre corrspondant à la plus petitte valeur propre
	double normalePoint[3];
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
		//cout << "vect = " << vecteurs[i][0] << " | " << vecteurs[i][1] << " | " << vecteurs[i][2] << endl;
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

	/*cout << "\n";
	cout << "TEST01\n";
	cout << "  For a symmetric matrix A,\n";
	cout << "  JACOBI_EIGENVALUE computes the eigenvalues D\n";
	cout << "  and eigenvectors V so that A * V = D * V.\n";
	r8mat_print ( n , n , matCov , "  Input matrix A:" );
	*/
	it_max = 100;

	jacobi_eigenvalue ( n, matCov, it_max, v, d, it_num, rot_num );
	/*
	cout << "\n";
	cout << "  Number of iterations = " << it_num << "\n";
	cout << "  Number of rotations  = " << rot_num << "\n";

	r8vec_print ( n, d, "  Eigenvalues D:" );

	r8mat_print ( n, n, v, "  Eigenvector matrix V:" );*/
	//  Compute eigentest.
	error_frobenius = r8mat_is_eigen_right ( n, n, matCov, v, d );
	/*cout << "\n";
	cout << "  Frobenius norm error in eigensystem A*V-D*V = " << error_frobenius << "\n";*/
	
	//paramètres à calculer
	double tau; //variation surfacique
	
	double tmpSum = d[0]+d[1]+d[2];
	double tmpMin = std::min( d[0] , d[1] );

	tmpMin = std::min( tmpMin , d[2] );
	
	//on cherche le vecteur propre correspondant
	if( tmpMin == d[0] )
	{
		normalePoint[0] = v[0];
		normalePoint[1] = v[1];
		normalePoint[2] = v[2];
	}
	if( tmpMin == d[1] )
	{
		normalePoint[0] = v[3];
		normalePoint[1] = v[4];
		normalePoint[2] = v[5];
	}
	if( tmpMin == d[2] )
	{
		normalePoint[0] = v[6];
		normalePoint[1] = v[7];
		normalePoint[2] = v[8];
	}
	
	
	/*cout << "somme des valeurs propres = " << tmpSum << "\n";
	cout << "plus petitte valeur propre = " << tmpMin << "\n";*/
	//on a donc ici nos valeurs/vecteurs propres
	
	//calcul de la "variation surfacique" tau, qui est une bonne approximation de la courbure
	tau = tmpMin / tmpSum;
	
	//on remplit nos resutat dans nos tableaux
	
	//cout << "NORMALE = " << normalePoint[0] << " | " << normalePoint[1] << " | " << normalePoint[2] << endl << endl;
	
	tabNormal[indiceTab][0] = normalePoint[0];
	tabNormal[indiceTab][1] = normalePoint[1];
	tabNormal[indiceTab][2] = normalePoint[2];
	
	
	tabTau[indiceTab] = tau;
	return;
}

double dotProduct( double v1 [3] , double v2 [3] )
{
	double res = 0.0;
	for( int i = 0 ; i < 3 ; i++ )
	{
		res = res + v1[i]*v2[i];
	}
	return res;
}

void normaliser( double A [3] )
{
	double normeA;
	normeA = sqrt( dotProduct( A , A ) );
	A[0] = A[0] / normeA;
	A[1] = A[1] / normeA;
	A[2] = A[2] / normeA;
}

void setDbl3( double A[3] , double B[3] ) //set tab A to values of tab B
{
	A[0] = B[0];
	A[1] = B[1];
	A[2] = B[2];
}

double calcAngle( double A[3] , double B[3] )
{
	double angle = 0.0;
       
	angle = dotProduct( A , B ) /  ( sqrt(dotProduct( A , A ))*sqrt(dotProduct( B , B )) ) ;
	//cout << "cos angle = " << angle << endl;
	if( angle > 1.0 )
		angle = angle - floor(angle);
	if( angle < -1.0 )
		angle = angle - ceil(angle);
	//cout << "cos angleNEW = " << angle << endl;
	angle = fabs(acos(angle));
	//cout << "angle = " << angle << endl;
	return angle;
}


//tau est la variation surfacique du point considérée, normalPoint sa normale (vecteur propre de sa plus dpetitte valeur propre), normalNeighbourgs celles de ses voisins, k le nb de voisins
// double[] tabAngle est le tableau des valeurs de tau de chaqu'une des points du nuage, a remplir à la case [indiceTab]
// double[] tabPoids est le tableau des valeurs de tau de chaqu'une des points du nuage, a remplir à la case [indiceTab]
void calculeAngleTau( double tau , double normalPoint[3] , double normalNeighbourgs[][3] , int k , double tabAngle [] ,  double tabPoids [] , int indiceTab )
{
	//poid du points considéré
	double poids = 0.0;
	double coefficiantTau; //coeff multiplicative appliqué a tau
	//on calcule la somme des angle entre la normale du point considéré et celle de ses voisins.
	double angle = 0.0;
	double tmpAngle = 0.0;
	double tmpPts [3] , tmpVoisin [3];
	setDbl3( tmpPts , normalPoint );
	normaliser( tmpPts );
	for( int i = 0 ; i < k ; i++ )
	{	
		tmpVoisin[0] = normalNeighbourgs[i][0];
		tmpVoisin[1] = normalNeighbourgs[i][1];
		tmpVoisin[2] = normalNeighbourgs[i][2];
		
		//cout << "Normale = " << normalNeighbourgs[i][0] << " | " << normalNeighbourgs[i][1] << " | " << normalNeighbourgs[i][2] << endl;
		normaliser( tmpVoisin );
		// tmpAngle = cos(angle) = v1 dotProduct v2 / norm(v1)*norm(v2)
		//tmpAngle = dotProduct( tmpPts , tmpVoisin );// / sqrt(dotProduct( normalPoint , normalPoint ))*sqrt(dotProduct( normalNeighbourgs[i] , normalNeighbourgs[i] )) ;
		//cout << "cos(angle) = " << tmpAngle << endl;
		tmpAngle = calcAngle( tmpPts , tmpVoisin );
		angle = angle + tmpAngle;
		//cout << "TMPangle = " << tmpAngle << endl;
	}
	//cout << "angle = " << angle << endl << endl;
	//on a ici notre angle et notre tau, on peux donc calculer le poid effectif de ce point.
	poids = angle + coefficiantTau*tau;
	
	tabAngle[indiceTab] = angle;
	tabPoids[indiceTab] = poids;
	return;
}
