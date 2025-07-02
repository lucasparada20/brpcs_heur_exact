#include "RemoveRandomBRPCS.h"

int RemoveRandomBRPCS::GetRandInt(int min, int max)
{
	if(min == max) return min;
	return (rand() % (max - min)) + min;
}

void RemoveRandomBRPCS::Remove(Sol & s, int count)
{
	//printf("RandomRemove\n");
	vect.clear(); vect.reserve( s.GetCustomerCount() );
	for(int i = 0 ; i < s.GetCustomerCount() ; i++)
		if( s.GetAssignedTo( s.GetCustomer(i) ) != NULL)
			vect.push_back( s.GetCustomer(i) );
	
	int cpt = std::min(count, (int)vect.size());
	for(int i = 0 ; i < cpt ; i++)
	{
		//int index = mat_func_get_rand_int(0, vect.size() - i);

		int index = GetRandInt(0, vect.size() - i);	
		s.RemoveAndUnassign( vect[index] );
		vect[index] = vect[ vect.size() - i - 1];
	}
	//printf("Cost after RandomRemove:%.2lf nbUnassigneds:%d\n",s.GetCost(),s.GetUnassignedCount());
};