#ifndef _CALCUL_TRANSFORMATIONS
#define _CALCUL_TRANSFORMATIONS
#include "calculTransformations.cpp"
extern double dot( double v1 [3] , double v2 [3] );
extern double calculerAngle( double A[3] , double B[3] );
extern void produitMatVect( double res [3] , double M [3][3] , double V[3] );
extern void R( double res[3] , double v [3] , double angle , double axe [3] , double rot[3][3] );
extern void prodVect( double res[3] , double u [3] , double v [3] );
extern void setTabDbl3( double A[3] , double B[3] );
extern void normalize( double A [3] );
extern void calculTransformation( double A [3] , double B [3] , double normaleA [3] , double normaleB [3] , double rotation [3][3], double translation [3] );
#endif /* #ifndef __include_fichier_h__ */
