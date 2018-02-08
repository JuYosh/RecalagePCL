#include "correspondance.hpp"

std::vector<std::vector<int> > correspondance(double angleS[], double angleM[], double courbureS[], double courbureM[], double seuilCourbure, double seuilAngle, std::vector<int> indicesS, std::vector<int> indicesM)
{
	int indiceMin;
	std::vector<int> bloap, tmp;
	std::vector<double> resultats;
	std::vector<std::vector<int> > correspondants;

	std::vector<int>::iterator i, j;

	for(i = indicesS.begin() ; i < indicesS.end() ; i++)
	{
		tmp.clear();
		bloap.clear();
		resultats.clear();

		for(j = indicesM.begin() ; j < indicesM.end() ; j++)
		{
			if(fabs(courbureS[*i] - courbureM[*j]) <= seuilCourbure && fabs(angleS[*i] - angleM[*j]) <= seuilAngle)
			{
				resultats.push_back(abs(courbureS[*i] - courbureM[*j]) + abs(angleS[*i] - angleM[*j])); // Une idée
				bloap.push_back(*j);
			}
		}
		
		if(!resultats.empty())
		{
			tmp.push_back(*i);
			indiceMin = bloap[min_element(resultats.begin(), resultats.end()) - resultats.begin()];
			tmp.push_back(indiceMin);

			correspondants.push_back(tmp);
		}
	}
	
	return correspondants;
}
