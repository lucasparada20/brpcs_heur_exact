
#ifndef EXACT_BRP_CALL_BACK_H
#define EXACT_BRP_CALL_BACK_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <vector>

#include "ExactBrpcsGraph.h"
#include "ExactBrpcsSepBase.h"
#include "../src_heur/Parameters.h"

class ExactBrpUserCutCallBackO : public IloCplex::UserCutCallbackI
{
	public:
		//ExactBrpUserCutCallBackO(IloEnv env, ExactBrpGraphO * graph, IloNumVarArray x, IloNumVar theta, ExactBrpSepO * sep);
		ExactBrpUserCutCallBackO(IloEnv env, ExactBrpGraphO * graph, IloNumVarArray x, IloNumVar theta, ExactBrpSepBase * sep);
		~ExactBrpUserCutCallBackO();

		IloCplex::CallbackI *duplicateCallback() const
		{
        	return new (getEnv()) ExactBrpUserCutCallBackO(*this);
        }

        void main();

	private:
    	ExactBrpGraphO * _graph;
    	IloNumVarArray _x;
		IloNumVar _theta;
    	//ExactBrpSepO * _sep;
		ExactBrpSepBase * _sep;
    public:
    	IloRangeArray added_constraints;
		bool add_constraints;
};

class ExactBrpLazyCallBackO : public IloCplex::LazyConstraintCallbackI
{
	public:
		//ExactBrpLazyCallBackO(IloEnv env, ExactBrpGraphO * graph, IloNumVarArray x, IloNumVar theta, ExactBrpSepO * sep);
		ExactBrpLazyCallBackO(IloEnv env, ExactBrpGraphO * graph, IloNumVarArray x, IloNumVar theta, ExactBrpSepBase * sep);
		~ExactBrpLazyCallBackO();

		IloCplex::CallbackI *duplicateCallback() const
		{
        	return new (getEnv()) ExactBrpLazyCallBackO(*this);
        }

        void main();

    private:
    	ExactBrpGraphO * _graph;
    	IloNumVarArray _x;
		IloNumVar _theta;
    	//ExactBrpSepO * _sep;
		ExactBrpSepBase * _sep;
    public:
    	IloRangeArray added_constraints;
		bool add_constraints;
};


#endif
