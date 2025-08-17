
#include "AlnsBRPCS.h"

void ALNS::Optimize(Sol & s)
{
	Optimize(s, NULL);
}

void ALNS::Optimize(Sol & s, BestSolutionList * best_sol_list)
{
	s.Update();
	Sol best = s;
	Sol cur = s;
	double best_cost = s.GetCost();
	if(best_sol_list != NULL && s.IsFeasible()) best_sol_list->Add(s);

	double init_dist = s.GetTotalDistances();
	double curr_cost = best_cost;

	/*_chrono.start();
	_chrono.setDuration(_max_time);*/

	Init();

	int nb_iter_without_new_best = 0;
	double w = 0.05;
	double Tmin = init_dist * (1+w) * pow(_temperature, _temperature_iter_init);
	//double Tmax = init_dist * (1+w) * pow(_temperature, 30000);
	double Tmax = 0.00000396;
	double T = Tmin;
	printf("ALNS it:%d Tmin:%.10lf TMax:%.10lf ",_iterator_count, Tmin,Tmax);
	printf("init_dist:%lf pow:%.10lf T:%.10lf\n",init_dist,pow(_temperature, 30000),_temperature);
	clock_t begin = clock();
	for(int iter = 0 ; iter < _iterator_count ;iter++)
	{
		int nbitems = s.GetCustomerCount() - s.GetUnassignedCount();
		int nbMaxRmv = std::min(_max_nb_items, (int)(nbitems*_pourcentage_max));
		int nbMinRmv = std::min(_min_nb_items, (int)(nbitems*_pourcentage_min));
		int nbremove =  (nbMinRmv == nbMaxRmv)? nbMaxRmv : rand() % nbMaxRmv + nbMinRmv;
		//if(Parameters::GetCostPolicy() == CN)
			nbremove = std::min(std::max(nbremove, 5),nbitems);
		//else 
			//nbremove = std::min(std::max(nbremove, 10),nbitems);
		
		alns_remove_operator * rmv = GetRemoveOperator();
		alns_insert_operator * ins = GetInsertOperator();

		ins->nb++;			rmv->nb++;
		ins->nb_selected++; rmv->nb_selected++;

		rmv->opt->Remove(cur, nbremove);
		cur.MakeFeasible(); //No need to call update if Insert comes immediately after. Insert calls update
		ins->opt->Insert(cur,false);

		double newcost = cur.GetCost();
		
		double gap_best = (newcost - best_cost) / best_cost;
		
		//if((iter % 1000) == 0)// || best_cost > newcost)
		{
			double time = (double)(clock() -  begin) / CLOCKS_PER_SEC;
			printf("Iter:%d rmv:%d newcost:%.2lf(%d,%d) cost:%.2lf best:%.2lf Drv:%d T:%.8lf time:%.1lf\n",
				iter,nbremove,newcost,cur.GetUnassignedCount(),(int)cur.IsFeasible(), curr_cost, best_cost, best.GetUsedDriverCount(), T, time);
				//printf("Unassigneds:");
				//for(int i=0;i<cur.GetUnassignedCount();i++)
				//	printf("%d ",cur.GetUnassigned(i)->id);
				//printf("\n");
		}
		

		if(best_sol_list != NULL && cur.IsFeasible())
			best_sol_list->Add(cur);
		
		/*if(Parameters::GetCostPolicy() == RT && (nb_iter_without_new_best % 500) == 0 && nb_iter_without_new_best)
		{
			cur.MergeAllPaths();
			newcost = cur.GetCost();
			printf("Merging paths ... newcost:%.1lf\n",newcost);
		}*/

		if(best_cost > newcost && cur.IsFeasible())
		{
			nb_iter_without_new_best = 0;
			best_cost = newcost;
			curr_cost = newcost;
			best = cur;
			s = cur;
			ins->score += _sigma1; 	rmv->score += _sigma1;
		}
		else
		{
			nb_iter_without_new_best++;
			double prob = rand() / (double)RAND_MAX;
			//double prob = mat_func_get_rand_double();
			double acpprob = exp((curr_cost-newcost)/T);

			
			if(prob < acpprob && gap_best <= _acceptation_gap)
			{
				//printf("prob:%lf acpprob:%lf T:%lf newcost:%lf prevcost:%lf best:%lf accept_gap:%.2lf\n", 
				//	prob, acpprob, T, newcost, curr_cost, best_cost, _acceptation_gap);
				s = cur;
				ins->score += newcost<curr_cost?_sigma2:_sigma3;
				rmv->score += newcost<curr_cost?_sigma2:_sigma3;
				curr_cost = newcost;
			}
			else
				cur = s;
		}//end else

		if(iter % 100 == 0) Update();
		/*if(iter % _chrono_check_iter == 0 && _chrono.hasToEnd()) break;*/

		T = T * _temperature;
		if((T < Tmax || T < 0.000001) 
			&& ( (nb_iter_without_new_best >= 1000 && Parameters::GetCostPolicy() == CN)
			|| (nb_iter_without_new_best >= 500 && Parameters::GetCostPolicy() == RT) ))
			T= Tmin;
	}//end for

	s = best;

}//end Optimize


void ALNS::AddInsertOperator(InsertOperator * opt)
{
	alns_insert_operator o;
	o.opt = opt;
	insert_operators.push_back(o);
}
void ALNS::AddRemoveOperator(RemoveOperator * opt)
{
	alns_remove_operator o;
	o.opt = opt;
	remove_operators.push_back(o);
}

void ALNS::Init()
{
	for(size_t j=0;j<insert_operators.size();j++)
	{
		insert_operators[j].w = 0;
		insert_operators[j].nb_selected=1;
		insert_operators[j].score=1;
		insert_operators[j].nb=0;
	}
	for(size_t j=0;j<remove_operators.size();j++)
	{
		remove_operators[j].w = 0;
		remove_operators[j].nb_selected=1;
		remove_operators[j].score=1;
		remove_operators[j].nb=0;
	}
}

void ALNS::Update()
{
	size_t j;
	for(j=0;j<insert_operators.size();j++)
	{
		insert_operators[j].w = 	insert_operators[j].w * (1 - _p) +
									insert_operators[j].score/insert_operators[j].nb_selected*_p;
		insert_operators[j].nb_selected=1;
		insert_operators[j].score=0;
	}
	for(j=0;j<remove_operators.size();j++)
	{
		remove_operators[j].w = 	remove_operators[j].w * (1 - _p) +
									remove_operators[j].score/remove_operators[j].nb_selected*_p;
		remove_operators[j].nb_selected=1;
		remove_operators[j].score=0;
	}
}

alns_remove_operator * ALNS::GetRemoveOperator()
{
	double sumw = 0;
	size_t i;
	for(i=0;i<remove_operators.size();i++)
		sumw += remove_operators[i].w;

	double interval = 0;
	for(i=0;i<remove_operators.size();i++)
	{
		remove_operators[i].interval1 = interval;
		interval += remove_operators[i].w/sumw;
		remove_operators[i].interval2 = interval;
	}

	double k = rand() / (double)RAND_MAX;
	for(i=0;i<remove_operators.size();i++)
		if(remove_operators[i].interval1 <= k && k <= remove_operators[i].interval2)
			return &remove_operators[i];
	return &remove_operators[remove_operators.size()-1];
}
alns_insert_operator * ALNS::GetInsertOperator()
{
	double sumw = 0;
	size_t i;
	for(i=0;i<insert_operators.size();i++)
		sumw += insert_operators[i].w;

	double interval = 0;
	for(i=0;i<insert_operators.size();i++)
	{
		insert_operators[i].interval1 = interval;
		interval += insert_operators[i].w/sumw;
		insert_operators[i].interval2 = interval;
	}

	double k = rand() / (double)RAND_MAX;
	for(i=0;i<insert_operators.size();i++)
		if(insert_operators[i].interval1 <= k && k <= insert_operators[i].interval2)
			return &insert_operators[i];
	return &insert_operators[insert_operators.size()-1];
}
