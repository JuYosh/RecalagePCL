#ifndef _CALCUL_PARAMETRES
#define _CALCUL_PARAMETRES
#include "calculParametres.cpp"
extern void calculParametre( double vecteurs [][3] , int k , double** tabNormal , double tabTau [] , int indiceTab  );
extern double dotProduct( double v1 [3] , double v2 [3] );
extern void calculeAngleTau ( double tau , double normalPoint[3] , double normalNeighbourgs[][3] , int k , double tabAngle [] ,  double tabPoids [] , int indiceTab );
#endif /* #ifndef __include_fichier_h__ */
