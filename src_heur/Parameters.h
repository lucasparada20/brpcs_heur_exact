
#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include <time.h>
#include <math.h>
#include <string>

class Parameters
{

public:
	
	static bool GetCostPolicy(){ return cost_policy; }
	static void SetCostPolicy(bool b){ cost_policy = b; }
	static char* GetInstanceFileName(){return instance_file.size() == 0?NULL:(char *)instance_file.c_str();}
	static char* GetReFileName(){ return (char *)re_file_name.c_str(); }
	static char* GetOutputFileName(){ return (char *)output_file_name.c_str(); }
	static char* GetCityName() {return (char*)city_name.c_str();}
	static int GetNbStations() {return nb_stations;}
	
	static int AddDepotStations(){ return add_depot_stations; }
	static void SetMaxRouteDistance(int d){ max_route_distance = d;}
	static int MaxRouteDistance(){ return max_route_distance; }
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
	
	static int max_route_distance;
	static int add_depot_stations;
	static int recourse_policy;
	static int delta;
	static int HardQ;
	static int nb_stations;
	static int u_value;
	
	static bool cost_policy;

};


#endif
