#ifndef LOAD_BRPCS_H
#define LOAD_BRPCS_H

#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include "ProblemDefinition.h"
#include "Parameters.h"
#include <map>

class LoadBRPCS {
public:
		void LoadInstance(Prob & pr, char * filename);
		double CalculateHarvesineDistance(Node * n1, Node * n2);
};

#endif