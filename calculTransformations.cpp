# include <cmath>
#include <iostream>
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
using namespace std;


double dot( double v1 [3] , double v2 [3] )
{
	double res = 0.0;
	for( int i = 0 ; i < 3 ; i++ )
	{
		res = res + v1[i]*v2[i];
	}
	return res;
}

double calculerAngle( double A[3] , double B[3] )
{
	double angle = 0.0;
	angle = dot( A , B ) / sqrt(dot( A , A ))*sqrt(dot( B , B )) ;
	//cout << "cos angle = " << angle << endl;
	if( angle > 1.0 )
		angle = angle - (int)(angle);
	if( angle < -1.0 )
		angle = angle - (int)(angle);
	//cout << "cos angleNEW = " << angle << endl;
	angle = fabs(acos(angle));
	//cout << "angle = " << angle << endl;
	return angle;
}

void produitMatVect( double res [3] , double M [3][3] , double V[3] )
{
	res[0] = dot( M[0] , V );
	res[1] = dot( M[1] , V );
	res[2] = dot( M[2] , V );
}

void R( double res[3] , double v [3] , double angle , double axe [3] , double rot[3][3] )
{
	//on remplit la matrice de rotation general par rapport a un axe 
	double a1 , a2 , a3;// on utilise 3 constantes pour ce calcul
	double a ,  b , c;
	a = axe[0];		b = axe[1];		c = axe[2];
	a1 = 1 - cos(angle);
	a2 = cos(angle);
	a3 = sin(angle);
	
	rot[0][0] = a1*a*a + a2;
	rot[0][1] = a1*a*b 	    - a3*c;
	rot[0][2] = a1*a*c 	    + a3*b;
	
	rot[1][0] = a1*b*a      + a3*c;
	rot[1][1] = a1*b*b + a2;
	rot[1][2] = a1*b*c      - a3*a;
	
	rot[2][0] = a1*c*a  	- a3*b;
	rot[2][1] = a1*c*b  	+ a3*a;
	rot[2][2] = a1*c*c + a2;
	
	produitMatVect( res , rot , v ); //si on veux verifier que les calculs sont corrects
}

void prodVect( double res[3] , double u [3] , double v [3] )
{
	res[0] = u[1]*v[2] - u[2]*v[1];
	res[1] = u[2]*v[0] - u[0]*v[2];
	res[2] = u[0]*v[1] - u[1]*v[0];
}
void setTabDbl3( double A[3] , double B[3] ) //set tab A to values of tab B
{
	A[0] = B[0];
	A[1] = B[1];
	A[2] = B[2];
}

void normalize( double A [3] )
{
	double normeA;
	normeA = sqrt( dot( A , A ) );
	A[0] = A[0] / normeA;
	A[1] = A[1] / normeA;
	A[2] = A[2] / normeA;
}
//calcul de la transformation (rotation et translation) pour passer de A à B
//On calcule la tranlation apr rapport aux coordonées des points et la rotation par rapport a leurs normales
//les resultats sont ecrits dans rotation et dans translation
void calculTransformation( double A [3] , double B [3] , double normaleA [3] , double normaleB [3] , double rotation [3][3], double translation [3] )
{
	double tmpAngle , angle;//angleX , angleY , angleZ;
	double tmpA [3];
	double tmpB [3];
	double axeRotation [3];
	double tmpRes [3];
	//A[0] = A[1] = A[2] = 1.0; 
	//B[0] = B[1] = B[2] = -1.0; 
	//calcul de la translation, on translate de A en B donc on applique le vecteur AB = B-A
	translation[0] = B[0] - A[0];
	translation[1] = B[1] - A[1];
	translation[2] = B[2] - A[2];
	
	//calcul de la rotation
	//on va calculer l'angle entre A et B
	setTabDbl3( tmpA , normaleA );
	setTabDbl3( tmpB , normaleB);
	normalize(tmpA);
	normalize(tmpB);
	angle = calculerAngle( tmpA , tmpB );
	/*cout << "vecteur A = " << normaleA[0] << " | "<< normaleA[1] << " | "<< normaleA[2] << endl;
	cout << "vecteur B = " << normaleB[0] << " | "<< normaleB[1] << " | "<< normaleB[2] << endl << endl;
	cout << "vecteur A norm = " << tmpA[0] << " | "<< tmpA[1] << " | "<< tmpA[2] << endl;
	cout << "vecteur B norm = " << tmpB[0] << " | "<< tmpB[1] << " | "<< tmpB[2] << endl << endl;*/
	//le vecteur portant l'axe de  la rotation est le produit vectoriel A vect B
	prodVect( axeRotation , tmpA , tmpB );
	normalize(axeRotation); //on normalise le vecteur portant l'axe de la rotation pour alleger le calcul
	R( tmpRes , tmpA , angle , axeRotation , rotation ); //on calcul la rotation que l'on met dans rotation
	
	/*cout << "angle = " << angle << endl;
	cout << "vecteur Rot(tmpA) = " << tmpRes[0] << " | "<< tmpRes[1] << " | "<< tmpRes[2] << endl;*/
}

/*double R()
{
	
	return (double)( rand() ) / (double)( RAND_MAX);
}


int main()
{
	srand (time(NULL));
	double A[3] , B[3] , rot[3][3] , trans[3];
	A[0] = R();		A[1] = R();		A[2] = R();
	B[0] = R();		B[1] = R();		B[2] = R();
	
	calculTransformation( A , B , rot , trans );
	
	return 0;
}*/
