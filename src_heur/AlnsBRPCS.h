
#ifndef ALNS_H
#define ALNS_H

#include "SequentialInsertionBRPCS.h"
#include "RelatednessRemoveBRPCS.h"
#include "RemoveRandomBRPCS.h"
#include "Solution.h"
#include "BestSolutionList.h"
#include "OperatorBase.h"
#include <math.h>

struct alns_remove_operator
{
	int no;
	double w;
	double nb_selected;
	double score;
	int nb;
	double interval1;
	double interval2;
	RemoveOperator * opt;
};

struct alns_insert_operator
{
	InsertOperator * opt;
	int no;
	double w;
	double nb_selected;
	double score;
	int nb;
	double interval1;
	double interval2;
};

class ALNS 
{
	public:
		ALNS()
		{
			_iterator_count = 25000;
			_temperature = 0.9996;
			_pourcentage_max = 0.4; 
			_pourcentage_min = 0.1;
			_max_nb_items = 30;	_min_nb_items = 5;
			_pourcentage_max = 0.05; _pourcentage_min = 0.01;
			_sigma1 = 4; _sigma2 = 1; _sigma3 = 0;
			_p = 0.05;
			_temperature_iter_init = 0;
			_acceptation_gap = 99999999;
			_max_time = 99999999;
			_chrono_check_iter = 1000;
			_recourse_policy = CN;
		}

		void Optimize(Sol & s);

		void Optimize(Sol & s, BestSolutionList * best_sol_list);

		void SetItemMinRemoved(int minp){_min_nb_items = minp;}
		void SetItemMaxRemoved(int maxp){_max_nb_items = maxp;}
		void SetPercentageMin(double minp){_pourcentage_min = minp;}
		void SetPercentageMax(double maxp){_pourcentage_max = maxp;}
		void SetTemperatureIterInit(double temp){_temperature_iter_init = temp;}
		void SetTemperature(double temp){_temperature = temp;}
		void SetIterationCount(int it){_iterator_count = it;}
		void SetMaxTime(int maxtime){_max_time = maxtime;}
		void SetChronoCheckIter(int iter){ _chrono_check_iter = iter;}
		void SetAcceptationGap(double a){_acceptation_gap = a;}
		void SetCostPolicy(int r){ _recourse_policy=r; }
		
		void AddInsertOperator(InsertOperator * opt);

		void AddRemoveOperator(RemoveOperator * opt);

	private:

		void Init();

		void Update();

		void DoPostAlnsAlgorithms(Sol & sol, BestSolutionList * best_sol_list);

		alns_remove_operator * GetRemoveOperator();
		alns_insert_operator * GetInsertOperator();


		int _iterator_count;
		double 	_temperature;
		double 	_pourcentage_max;
		double 	_pourcentage_min;
		int 	_max_nb_items;
		int 	_min_nb_items;
		double 	_sigma1;
		double 	_sigma2;
		double 	_sigma3;
		double 	_p;
		double _temperature_iter_init;
		double _acceptation_gap;
		std::vector<alns_insert_operator > insert_operators;
		std::vector<alns_remove_operator > remove_operators;
		int _max_time;			//max resolution time
		int _chrono_check_iter; //number of iteration before a check of the chrono (might cost alot)
		int _recourse_policy;

};

#endif
