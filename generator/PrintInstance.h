#ifndef PRINT_INSTANCE_H
#define PRINT_INSTANCE_H

#include "Station.h"
#include <string>
#include <stdio.h>

void PrintInstance(std::vector<Station> & stations, std::string & city_name, double uncharged_percentage)
{
    int uncharged_int = 100*uncharged_percentage;
	std::string filename = city_name + std::to_string((int)stations.size()) + "_" +std::to_string(uncharged_int) + ".txt"; //with integer to string you need std::to_string() ...
    FILE* output_file = fopen(filename.c_str(), "w");
    if (!output_file) {
        std::cerr << "Error: Unable to open the single station output file." << std::endl;
        return;
    }
	printf("Printing output stations to file:%s\n",filename.c_str());
	
	int cntr = 1; int sumCap = 0; int sumqReg = 0; int sumAbsReg = 0; int sumqElec = 0; int sumAbsElec = 0; int sumInitReg = 0; int sumInitElec = 0; int sumInitU = 0; int charges = 0;
	for (const Station& s : stations)
		if( s.tgtElectric > -1 && s.tgtRegular > -1  )
		{
			cntr++; sumCap += s.cap;
			sumInitReg += s.initRegular; sumInitElec += s.initElectric; sumInitU += s.uElectric;
			sumqReg += s.initRegular - s.tgtRegular; sumqElec += s.initElectric - s.tgtElectric;
			sumAbsReg += std::abs( s.initRegular - s.tgtRegular); sumAbsElec += std::abs(s.initElectric -  s.tgtElectric);
			if(s.charges) charges++;
		}
    fprintf(output_file, "%d %d %d %d %d %d %d %d %d %d\n", 
			cntr, charges, sumCap, sumInitReg, sumInitElec, sumInitU, sumqReg, sumqElec, sumAbsReg, sumAbsElec);

    for (const Station& s : stations)
	{
		if(s.name == "depot")
			fprintf(output_file, "%d %.6lf %.6lf\n",s.cap,s.lat,s.lon);
		else if( s.tgtElectric > -1 && s.tgtRegular > -1  )
		{
			fprintf(output_file,"%d %d %d %d %d %d %d %.6lf %.6lf\n"
								,s.cap,s.charges,s.tgtRegular,s.tgtElectric
								,s.initRegular,s.initElectric,s.uElectric
								,s.lat,s.lon);
								
			if(s.initRegular + s.initElectric + s.uElectric > s.cap)
			{
				s.Show(); printf("Wrong cap while printing instance ... Exiting.\n"); exit(1);
			}
		}
			
	}
    fclose(output_file);
	printf("1st line of the instances \nnbStat:%d charStat:%d sumCap:%d sumInitReg:%d sumInitElec:%d sumInitU:%d sumqReg:%d sumqElec:%d sumAbsReg:%d sumAbsElec:%d\n",
		cntr, charges, sumCap, sumInitReg, sumInitElec, sumInitU, sumqReg, sumqElec, sumAbsReg, sumAbsElec);
}

#endif