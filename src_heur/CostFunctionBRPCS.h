#ifndef COST_FUNCTION_BRPCS	
#define COST_FUNCTION_BRPCS

#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include "ProblemDefinition.h"
#include "constants.h"
#include "Parameters.h"
#include "RouteFeasibilityBRPCS.h"

#include <cstring> // For strcmp

class Sol;

class CostFunctionBRPCS
{
	public:
		CostFunctionBRPCS(RouteFeasibility * r) : _r(r){};

		double GetCost(Sol & s);
		double GetCost(Sol & s, Driver * d);
		void Update(Sol & s);
		void Update(Sol & s, Driver * d);
		void Show(Sol * s, Driver * d);
		void Show(Sol * s, Driver * d, int & total_route_cost);
		
	private:
		std::vector<Node*> path;
		RouteFeasibility * _r;
		std::vector<int> path_ids;
		
};


#endif
