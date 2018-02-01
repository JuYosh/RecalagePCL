#include "correspondance.hpp"

std::vector<std::vector<int> > correspondance(float angleS[], float angleM[], float courbureS[], float courbureM[], float seuilCourbure, float seuilAngle, std::vector<int> indicesS, std::vector<int> indicesM)
{
	int indiceMin, k;
	std::vector<int> bloap;
	std::vector<float> resultats;
	std::vector<std::vector<int> > correspondants;

	k = 0; // Indice dans le vecteur de correspondances
	for(std::vector<int>::iterator i = indicesS.begin() ; i != indicesS.end() ; i++)
	{
		bloap.clear();
		resultats.clear();
		correspondants[k].push_back(i);
		
		for(std::vector<int>::iterator j = indicesM.begin() ; j != indicesM.end() ; j++)
		{
			if(abs(courbureS[i] - courbureM[j]) <= seuilCourbure && abs(angleS[i] - angleM[j]) <= seuilAngle)
			{
				resultats.push_back(abs(courbureS[i] - courbureM[j]) + abs(angleS[i] - angleM[j])); // Une idée
				bloap.push_back(j);
			}
		}
		
		if(!resultats.empty())
		{
			indiceMin = bloap[min_element(resultats.begin(), resultats.end()) - resultats.begin()];
			correspondants[k].push_back(indiceMin);
		}
		else
			correspondants[k].push_back(-1);
		
		k++;
	}
	
	return correspondants;
}
