#include "ExactBrpcsCallBacks.h"

ExactBrpUserCutCallBackO::ExactBrpUserCutCallBackO(IloEnv env, ExactBrpGraphO * graph,
													IloNumVarArray x, IloNumVar theta, ExactBrpSepBase * sep) :
													IloCplex::UserCutCallbackI(env), _graph(graph), _x(x),_theta(theta),_sep(sep),
													added_constraints(env),add_constraints(false)
{

}
ExactBrpUserCutCallBackO::~ExactBrpUserCutCallBackO()
{
	added_constraints.end();
}
void ExactBrpUserCutCallBackO::main()
{
	//printf("ExactSbrpUserCutCallBackO obj:%.2lf\n", (double)getObjValue());
	if(!isAfterCutLoop()) return;

	IloNumArray values(getEnv());
	getValues(values,_x);
	for(int i = 0 ; i < _graph->GetArcCount();i++)
	{
		ExBrpArcO * arc = _graph->GetArc(i);
		arc->value = (double)values[arc->index];
	}
	values.end();
	_graph->AssignPositiveValues();
	
	IloRangeArray inq(getEnv());
	_sep->SeparateFrac(inq);
	
	if(inq.getSize() >= 1 && add_constraints)
		added_constraints.add(inq);
		
	for(int i=0;i<inq.getSize();i++)
	{
		add(inq[i]);
	}
	inq.end();
}


ExactBrpLazyCallBackO::ExactBrpLazyCallBackO(IloEnv env, ExactBrpGraphO * graph,
											 IloNumVarArray x, IloNumVar theta, ExactBrpSepBase * sep) :
											 IloCplex::LazyConstraintCallbackI(env), _graph(graph), _x(x),_theta(theta),_sep(sep),
											 added_constraints(env),add_constraints(false)
{

}
ExactBrpLazyCallBackO::~ExactBrpLazyCallBackO()
{
	added_constraints.end();
}
void ExactBrpLazyCallBackO::main()
{
	//printf("ExactBrpLazyCallBackO obj:%.2lf\n", (double)getObjValue());
	IloNumArray values(getEnv());
	getValues(values,_x);
	for(int i = 0 ; i < _graph->GetArcCount();i++)
	{
		ExBrpArcO * arc = _graph->GetArc(i);
		arc->value = (double)values[arc->index];
	}
	values.end();

	_graph->AssignPositiveValues();
	//_graph->ShowPosValueArcs();

	IloRangeArray inq(getEnv());
	_sep->SeparateInt(inq);
	
	//_graph->MakePaths(); //It is done iff no subtour is found

	if (inq.getSize() == 0)
		_sep->SeparateOptCut(inq);

	if(inq.getSize() >= 1 && add_constraints)
		added_constraints.add(inq);
	for(int i=0;i<inq.getSize();i++)
		add(inq[i]);

	//printf("CallBack: dist:%.1lf t:%.1lf inqs:%d\n",_graph->GetCost(),getValue(_theta),(int)inq.getSize());
	inq.end();
}

