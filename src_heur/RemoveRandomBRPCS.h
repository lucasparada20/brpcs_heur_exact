#ifndef REMOVE_RANDOM
#define REMOVE_RANDOM

#include "Solution.h"
#include "OperatorBase.h"

#include <vector>
#include <algorithm>
  
class RemoveRandomBRPCS : public RemoveOperator
{
	public:
		RemoveRandomBRPCS(){}
		~RemoveRandomBRPCS(){}	
	
		void Remove(Sol & s, int count) override;
		int GetRandInt(int min, int max);

	private:
		std::vector<Node*> vect;
		
};


#endif
