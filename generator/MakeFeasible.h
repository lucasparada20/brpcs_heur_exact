#ifndef MAKE_FEASIBLE_H
#define MAKE_FEASIBLE_H

#include "Station.h"
#include <stdio.h>
#include <vector>
#include <iostream>

void MakeFeasible(std::vector<Station> &stations) 
{
	int modifiedStations = 0;
	int modifiedCap = 0;
	int sumDiffCap = 0;
	int tgt = 0;
    
	for (size_t i = 0; i < stations.size(); i++) 
	{
        Station &s = stations[i];
        int q_i = s.initRegular - s.tgtRegular;
        int q_e = s.initElectric - s.tgtElectric;
        bool is_okay = false;
		
		if(s.initRegular<0) s.initRegular = 0;
		if(s.initElectric<0) s.initElectric = 0;
		if(s.uElectric<0 || s.charges == 1) s.uElectric = 0;
		
		if (s.cap < s.initRegular + s.uElectric + s.initElectric)
		{
			sumDiffCap +=  (s.initRegular + s.uElectric + s.initElectric) - s.cap;
			s.cap = s.initRegular + s.uElectric + s.initElectric; 
			modifiedCap++;
		}
		

        if (0 <= s.initRegular - q_i + s.initElectric - q_e + s.uElectric 
			&& s.initRegular - q_i + s.initElectric - q_e + s.uElectric <= s.cap 
			&& std::abs(q_i) + std::abs(q_e) <= s.cap)
            is_okay = true;
			
		if(!is_okay) modifiedStations++;
		
		int modifiedCount = 0;
		if (modifiedCount > 1000) {
			std::cerr << "Warning: Could not make station feasible after 1000 attempts. Manually modifying ..." << std::endl;

			// Calculate remaining capacity after considering uElectric
			int remainingCapacity = s.cap - s.uElectric;

			// Distribute remaining capacity between tgtRegular and tgtElectric
			if (remainingCapacity >= 0) {
				s.tgtRegular = std::max(0, s.initRegular - remainingCapacity / 2);
				s.tgtElectric = std::max(0, s.initElectric - remainingCapacity / 2);
			} else {
				s.tgtRegular = 0;
				s.tgtElectric = 0;
			}

			// Recompute q_i and q_e after manual adjustment and re-check feasibility
			int q_i = s.initRegular - s.tgtRegular;
			int q_e = s.initElectric - s.tgtElectric;
			
			if (0 <= s.initRegular - q_i + s.initElectric - q_e + s.uElectric &&
				s.initRegular - q_i + s.initElectric - q_e + s.uElectric <= s.cap &&
				std::abs(q_i) + std::abs(q_e) <= s.cap) {
				is_okay = true;
			} else {
				std::cerr << "Manual modification failed to make the station feasible." << std::endl;
				exit(1);
			}
		}

    }
	
	for(size_t i=0;i<stations.size();i++)
	{
		Station & s = stations[i];
		if(s.tgtRegular > 100 || s.tgtElectric > 100)
		{
			tgt++;
			stations.erase(stations.begin() + i);
		}
			
	}
	
	printf("Modified tgt:%d cap:%d diffCap:%d deleted:%d out of %d total stations, in MakeFeasible!\n",
		modifiedStations, modifiedCap, sumDiffCap, tgt, (int)stations.size());
}

#endif