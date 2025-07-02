#ifndef OPERATOR_BASE
#define OPERATOR_BASE

#include "Solution.h"

class RemoveOperator
{
	public:
		RemoveOperator(){}
		virtual ~RemoveOperator(){}	
		
		//Removes count number of customers
		virtual void Remove(Sol & s, int count) = 0;
};

class InsertOperator
{
	public:
		InsertOperator(){}
		virtual ~InsertOperator(){}
		
		//Try to insert all unassigned customers
		virtual void Insert(Sol & s, bool show) = 0;
};

#endif