
#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include <time.h>
#include <math.h>
#include <string>
#include <cstring>

#define CS 1
#define SW 2

#define CN 0
#define RT 1

class Parameters
{
	public:
		
		static int GetCostPolicy(){ return cost_policy; }
		static void SetCostPolicy(int b){ cost_policy = b; }
		static char* GetInstanceFileName(){return instance_file.size() == 0?NULL:(char *)instance_file.c_str();}
		static char* GetReFileName(){ return (char *)re_file_name.c_str(); }
		static char* GetOutputFileName(){ return (char *)output_file_name.c_str(); }
		static char* GetCityName() {return (char*)city_name.c_str();}
		static int GetNbStations() {return nb_stations;}
		
		static char* GetInitialSolutionFileName(){ return (char*)initial_solution_file.c_str(); }
		static char* GetConstructionHeuristic(){ return (char*)construction_heuristic.c_str(); }
		static void SetBSSType(char * s){ 
			if(strcmp(s,"CS")==0) bss_type = CS;
			if(strcmp(s,"SW")==0) bss_type = SW;
		}
		static int GetBSSType() { return bss_type; }
		static int AddDepotStations(){ return add_depot_stations; }
		static void SetMaxRouteDistance(double d){ max_route_distance = d;}
		static double MaxRouteDistance(){ return max_route_distance; }
		static void SetRecoursePolicy(int p){ recourse_policy = p; }
		static int RecoursePolicy(){ return recourse_policy; }
		static int GetDelta(){ return delta; }
		static void SetDelta(int d){ delta=d; }
		static int GetHardQ(){ return HardQ; }
		static void SetHardQ(int q){ HardQ = q; }
		static int GetUValue(){ return u_value; }
		
		static void SetCityName(const char* filePath);
		
		//Function to read all input parameters
		void Read(int arg, char ** argv);
		
	private:

		static std::string instance_file;
		static std::string re_file_name;
		static std::string output_file_name;
		static std::string city_name;
		static std::string construction_heuristic;
		static std::string initial_solution_file;
		
		static int bss_type;
		static double max_route_distance;
		static int add_depot_stations;
		static int recourse_policy;
		static int delta;
		static int HardQ;
		static int nb_stations;
		static int u_value;
		
		static int cost_policy;

};


#endif
