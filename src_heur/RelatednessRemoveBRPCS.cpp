#include "RelatednessRemoveBRPCS.h"
#include <algorithm>

void RelatednessRemoveBRPCS::Remove(Sol & s, int count)
{
	//printf("RemoveRelated\n");
	vect.clear(); vect.reserve( s.GetCustomerCount() );
	removed.clear(); removed.reserve( s.GetCustomerCount() );
	for(int i = 0 ; i < s.GetCustomerCount() ; i++)
	{
		if( s.GetAssignedTo( s.GetCustomer(i) ) != NULL)
			vect.push_back(s.GetCustomer(i));
		else
			removed.push_back(s.GetCustomer(i));
	}
	if(vect.size() <= 1) return;

	int size = vect.size() - 1;
	
	int index = vect.size() == 0 ? 0 : (rand() % vect.size());
	
	s.RemoveAndUnassign( vect[index] );
	removed.push_back(vect[index]);
	vect[index] = vect[size];

	RelatedPairSorter mysorter;
	mysorter.rel = this; //Already an instance

	int cpt = std::min(count-1, (int)vect.size()-2);
	for(int i = 0 ; i < cpt ; i++)
	{
		
		int index = removed.size() == 0 ? 0 : (rand() % removed.size());
		mysorter.selectedNode = removed[index];

		std::sort(vect.begin(), vect.begin()+size, mysorter);

		double pRelated = 6; //The value is set equal to that chosen by Ropke and Pisinger. Note that it might also be a random number
		
		index = (int)size * pow( rand() / (double)RAND_MAX, pRelated );
		if(index >= size) index--;

		s.RemoveAndUnassign( vect[index] );
		removed.push_back(vect[index]);
		vect[index] = vect[size-1];
		size--;
	}

}