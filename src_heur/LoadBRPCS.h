#ifndef LOAD_BRPCS_H
#define LOAD_BRPCS_H

#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include "ProblemDefinition.h"
#include "Parameters.h"
#include <map>
#include "Solution.h"

class LoadBRPCS {
public:
		void LoadInstance(Prob & pr, char * filename);
		double CalculateHarvesineDistance(Node * n1, Node * n2);
		void LoadSolution(Prob & pr, Sol & sol, char * filename);
};

#endif