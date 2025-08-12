#ifndef MAKE_UNCHARGED_SW_H
#define MAKE_UNCHARGED_SW_H

#include <vector>
#include <stdio.h>
#include "Station.h"
#include "RandomNumbers.h"

void MakeUnchargedSW(std::vector<Station> & stations, double uncharged_percentage, RandomNumbers & rn)
{
    int sum_electric = 0;
    int sum_uelectric = 0;
	int max_available_cap = 0;
	
	std::vector<int> indices; indices.reserve( (int)stations.size() );
    for (size_t i = 0; i < stations.size(); i++)
	{
		sum_electric += stations[i].initElectric;
		if(stations[i].charges)
			stations[i].charges = false;
		
		indices.emplace_back( i );
		max_available_cap += stations[i].cap;
	}
        
    if (sum_electric == 0) 
	{
		printf("Phil Collins (1989). No electric bikes. Exiting ...\n"); exit(1);
	}

	printf("Making %d%% of e/u in %d stations from %d electric bikes and max_available_cap:%d\n",(int)(uncharged_percentage*100),(int)indices.size(),sum_electric,max_available_cap);
	
	
    for (size_t i = 0; i < stations.size(); i++)
        stations[i].uElectric = 0;
	
	// Sequentially assign uncharged bikes
	int cntr = 0, iter = 0;
	int updates_this_cycle = 0;
	double curr_percentage = 0.0;

	while (std::abs(curr_percentage - uncharged_percentage) > 0.005)
	{
		Station &s = stations[indices[cntr]];

		int idx = -1;
		idx = rn.randInt(1, stations.size() - 1); // No depot
		Station &s1 = stations[idx];

		bool action_taken = false;

		if (curr_percentage < uncharged_percentage &&
			s.initRegular + s.uElectric + s.initElectric + 1 <= s.cap &&
			s1.initRegular + s1.uElectric + s1.initElectric - 1 >= 0)
		{
			s.uElectric++;
			sum_uelectric++;
			s1.initElectric--;
			sum_electric--;
			updates_this_cycle++;
			action_taken = true;
		}
		else if (curr_percentage > uncharged_percentage &&
				 s1.initRegular + s1.uElectric + s1.initElectric + 1 <= s1.cap &&
				 s.initRegular + s.uElectric + s.initElectric - 1 >= 0)
		{
			s.uElectric--;
			sum_uelectric--;
			s1.initElectric++;
			sum_electric++;
			updates_this_cycle++;
			action_taken = true;
		}

		curr_percentage = sum_uelectric / (double)sum_electric;

		iter++;

		// To print
		//if (action_taken)
		//{
		//	printf("[Iter:%d] Moved 1 bike | From station idx:%d to station idx:%d\n", iter, idx, indices[cntr]);
		//	printf("After move: sumU:%d initElectric:%d u/e:%.5lf target_u/e:%.5lf\n",
		//		   sum_uelectric, sum_electric, curr_percentage, uncharged_percentage);
		//}
		//else
		//{
		//	printf("[Iter:%d] No action taken, skipping...\n", iter);
		//}

		// Print every 50 iterations for more global status
		if (iter % 50 == 0)
		{
			printf(">>> Iteration checkpoint: Iter:%d sumU:%d initElectric:%d u/e:%.3lf cntr:%d total stations:%d updates_this_cycle:%d\n",
				   iter, sum_uelectric, sum_electric, curr_percentage,
				   cntr, (int)indices.size(), updates_this_cycle);
			// getchar(); // optional
			if(iter > 20000)
			{
				printf("Infinite loop. Phil Collins (1989). Exiting ...\n"); exit(1);
			}
		}

		cntr = (cntr + 1) % indices.size();

		if (cntr == 0)
		{
			if (updates_this_cycle == 0)
			{
				printf("No more stations with usableElectric > 0 — breaking to avoid infinite loop.\n");
				break;
			}
			updates_this_cycle = 0;
		}
	}



	printf("Final sum of uElectric:%d initElectric:%d u/e:%.3lf in SW\n",
			sum_uelectric,sum_electric, sum_uelectric/(double)sum_electric);
}

#endif