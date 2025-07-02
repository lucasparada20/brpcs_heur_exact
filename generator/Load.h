#ifndef LOAD_H
#define LOAD_H

#include "json.hpp"
#include <vector>
#include <fstream>
#include <stdio.h>
#include <map>
#include "RandomNumbers.h"

void UpdateQuantities(std::string & city_name, const nlohmann::json & station,
						int & sum_init_q, int & sum_init_qe, int & last_init_q, int & last_init_qe)
{
	// Check the different formats in json status file
	// The station will be added only if the below 'if' is satisfied ...
	if( (station["is_returning"] == 1 || station["is_returning"] == true) && (station["is_renting"] == 1 || station["is_renting"] == true) && (station["is_installed"] == 1 || station["is_installed"] == true) )
	{
		//std::cout << "city_name:" << city_name << " is operational!" << std::endl;
		// Madrid, Québec, Rio, Toronto, Vancouver, etc
		if( city_name == "madrid" || city_name == "quebec" || city_name == "rio" || city_name == "toronto" || city_name == "newyork" || city_name == "antwerp" || city_name == "washington" || city_name == "sanfrancisco" || city_name == "lyon" || city_name == "chicago" || city_name == "barcelona" || city_name == "bruxelles" || city_name == "saopaulo" || city_name == "santiago")
		{
			//std::cout << "City_name: " << city_name << " Full station JSON:\n" << station.dump(2) << std::endl;
			for( const nlohmann::json & vehicle : station["vehicle_types_available"] ) 
			{
				std::string type = vehicle["vehicle_type_id"];
				
				if (type == "ICONIC" || type == "FIT" || type == "1" || type == "bike" || type == "mechanical" || type == "0" || type == "150" || type == "BATTLE_MEC" || type == "CC") 
				{
					sum_init_q += vehicle.value("count", 0); // Default to 0 if missing;
					last_init_q = vehicle.value("count", 0);
				} 
				if (type == "BOOST" || type == "EFIT" || type == "BATTLE" || type == "EFIT G5" || type == "2" || type == "ebike" || type == "electrical" || type == "EFIT LARANJA" || type == "EFIT LARANJINHA" || type == "MUVO" || type == "OKAI" || type == "PEDALLA") 
				{
					sum_init_qe += vehicle.value("count", 0); // Default to 0 if missing;
					last_init_qe = vehicle.value("count", 0);
				}
				//std::cout << "Full vehicle JSON:\n" << vehicle.dump(2) << "\n";
			}
		}
		// Montreal, Boston
		if( city_name == "montreal" || city_name == "boston" )
		{
			sum_init_q += std::max(0,(int)station["num_bikes_available"]);
			sum_init_qe += std::max(0,(int)station["num_ebikes_available"]);
			
			last_init_q = std::max(0,(int)station["num_bikes_available"]);
			last_init_qe = std::max(0,(int)station["num_ebikes_available"]);
		}
		// Paris
		if( city_name == "paris")
		{
			for(const nlohmann::json & vehicle : station["num_bikes_available_types"])
			{
				// ToDo -> Should ensure : 0 <= count <= station_capacity ...
				if( vehicle.contains("mechanical") )
				{
					sum_init_q += vehicle["mechanical"].get<int>(); // Default to 0 if missing;
					last_init_q = vehicle["mechanical"].get<int>();
				}
				if( vehicle.contains("ebike") )
				{
					sum_init_qe += vehicle["ebike"].get<int>(); // Default to 0 if missing;
					last_init_qe = vehicle["ebike"].get<int>();
				}
			}
		}
		if( city_name == "mexicocity")
		{
			sum_init_q += 0;
			last_init_q = 0;
			
			int count = station["num_bikes_available"].get<int>() - station["num_bikes_disabled"].get<int>();
			sum_init_qe += count >= 0 ? count : 0;
			last_init_qe = count >= 0 ? count : 0;
		}
		
	}	
}

void LoadJson(std::string& city_name,
              std::string& last_station_information,
              std::vector<std::string>& station_status_file_names,
              std::vector<Station>& stations,
              RandomNumbers& rn)
{
    std::ifstream file(last_station_information);
    if (!file.is_open()) {
        printf("File %s not open. Phil Collins (1989). Exiting\n", last_station_information.c_str());
        exit(1);
    }

    nlohmann::json data;
    file >> data;
    auto json_stations = data["data"]["stations"];

    int cntr = 1;
    std::vector<Station> stations_temp; stations_temp.reserve(3000);
    std::vector<double> u_vec(3000);
    for (auto& u : u_vec) u = rn.rand01();

    printf("Parsing %s ...\n", last_station_information.c_str());
    for (const auto& item : json_stations) {
        int cap = 0;
		if(item.contains("capacity"))
			cap = item["capacity"];
        
		double lon = item["lon"];
        double lat = item["lat"];
		
		std::string bss_station_id;
		if(city_name != "paris")
			bss_station_id = item["station_id"];
        else 
			bss_station_id = std::to_string(item["station_id"].get<int>()); // its a number in paris
		
		std::string name = item["name"];
        bool charges = false;

        if (item.contains("is_charging_station") && city_name != "mexicocity" && city_name != "barcelona" && city_name != "saopaulo" && city_name != "santiago")
            charges = item["is_charging_station"];
        else if (item.contains("is_charging") && city_name != "mexicocity" && city_name != "barcelona" && city_name != "saopaulo" && city_name != "santiago")
            charges = item["is_charging"];
        else
            charges = (u_vec[cntr] <= 0.2);

        stations_temp.emplace_back(cntr, cap, charges, bss_station_id, name, lat, lon);
		stations_temp[stations_temp.size()-1].Show(); //getchar();
        cntr++;
    }

    // Preload all status data
	printf("Preloading all station status files ... \n");
    std::map<std::string, std::vector<nlohmann::json>> station_status_map;
    for (const auto& filename : station_status_file_names) {
        std::ifstream file_status(filename);
        if (!file_status.is_open()) {
            printf("File %s not open. Phil Collins (1989). Exiting\n", filename.c_str());
            exit(1);
        }

        nlohmann::json data_status;
        file_status >> data_status;

        for (const auto& item : data_status["data"]["stations"]) {
            
			std::string id;
			if(city_name != "paris")
				id = item["station_id"];
			else 
				id = std::to_string(item["station_id"].get<int>());
			
            station_status_map[id].emplace_back(item);
        }
    }

    int modifiedTgt = 0;
    printf("Initially loaded stations: %zu\n", stations_temp.size());

    for (size_t i = 0; i < stations_temp.size(); ++i) {
        Station& s = stations_temp[i];

        if (i % 50 == 0)
            printf("Parsing station: %zu\n", i);

        auto it = station_status_map.find(s.bss_station_id);
        if (it == station_status_map.end()) continue;

        int sum_q = 0, last_q = 0;
        int sum_qe = 0, last_qe = 0;
        int local_cntr = 0;

        for (const auto& entry : it->second) {
            UpdateQuantities(city_name, entry, sum_q, sum_qe, last_q, last_qe);
            local_cntr++;
        }

        if (local_cntr > 0) {
            s.tgtRegular = sum_q / (double)(local_cntr);
            s.tgtElectric = sum_qe / (double)(local_cntr);
            s.initRegular = last_q;
            s.initElectric = last_qe;

            if (s.cap < s.initRegular + s.initElectric)
                s.cap = s.initRegular + s.initElectric;
        }

        if (s.tgtRegular > s.cap || s.tgtElectric > s.cap) {
            printf("Station %d with greater tgt than cap ... adjusting cap ... \n", s.id);
            s.Show();
            s.cap = std::max(s.tgtRegular, s.tgtElectric) + 1;
            modifiedTgt++;
        }
    }

    // Final filtering
    stations.reserve(3000);
    for (const auto& s : stations_temp) {
        if (s.tgtRegular >= 0 && s.tgtElectric >= 0)
            stations.push_back(s);
    }

    int sum_q = 0, sum_qe = 0;
    for (const auto& s : stations) {
        sum_q += s.initRegular;
        sum_qe += s.initElectric;
    }

    printf("Loaded %zu stations with sum_q: %d sum_qe: %d. ModifiedTgts: %d\n",
           stations.size(), sum_q, sum_qe, modifiedTgt);
}


void LoadJsonSlow(std::string & city_name, std::string & last_station_information, std::vector<std::string> & station_status_file_names, 
              std::vector<Station> & stations, RandomNumbers & rn)
{
    std::ifstream file(last_station_information);
    if(!file.is_open())
    {
        printf("File %s not open. Phil Collins (1989). Exiting\n", last_station_information.c_str());
        exit(1);
    }

    nlohmann::json data;
    file >> data;
    nlohmann::json json_stations = data["data"]["stations"];

    int cntr = 1;
	std::vector<Station> stations_temp; stations_temp.reserve(3000);
	std::vector<double> u_vec; u_vec.reserve(3000);
	for(size_t i=0;i<3000;i++)
		u_vec.emplace_back( rn.rand01());
	
	printf("Parsing %s ...\n",last_station_information.c_str());
    for(const auto & item : json_stations)
    {
        int cap = item["capacity"];
        //if(cap == 0) continue;
		//if(cap == 0)
			//printf("item:%s cap:%d",item["station_id"], cap);
		
		double lon = item["lon"];
        double lat = item["lat"];
        std::string bss_station_id = item["station_id"];
        std::string name = item["name"];
		bool charges = false;
		if(item.contains("is_charging_station"))
			charges = item["is_charging_station"];
		else if(item.contains("is_charging"))
			charges = item["is_charging"];
		else // !item.contains("is_charging_station") && !item.contains("is_charging")
		{
			if(u_vec[cntr] <= 0.2)
				charges = true;
			else 
				charges = false;
		}

        stations_temp.emplace_back(cntr, cap, charges, bss_station_id, name, lat, lon);
        stations_temp[stations_temp.size()-1].Show(); //getchar();
		cntr++;
    }

   int modifiedTgt = 0;
   printf("Initially loaded stations: %d\n",stations_temp.size());
    for(size_t i = 0; i < stations_temp.size(); i++)
    {
        Station & s = stations_temp[i];
        int sum_init_q = 0; int last_init_q = 0;
        int sum_init_qe = 0; int last_init_qe = 0;
        int local_cntr = 0;
		
		if(i % 50 == 0)
			printf("Parsing station: %d\n",i);
        for(size_t j = 0; j < station_status_file_names.size(); j++)
        {
            std::ifstream file_status(station_status_file_names[j]);
            if(!file_status.is_open())
            {
                printf("File %s not open. Phil Collins (1989). Exiting\n", station_status_file_names[j].c_str());
                exit(1);
            }
				
            nlohmann::json data_status;
            file_status >> data_status;
            nlohmann::json json_stations = data_status["data"]["stations"];

            for(const auto & item : json_stations)
            {
                if(s.bss_station_id == std::string(item["station_id"]))
                {
                    UpdateQuantities(city_name, item, sum_init_q, sum_init_qe, last_init_q, last_init_qe);
                    local_cntr++;
                    break;
                }
            }
        }

        if(local_cntr)
        {
            s.tgtRegular = sum_init_q / (double)(local_cntr);
            s.tgtElectric = sum_init_qe / (double)(local_cntr);
			
			s.initRegular = last_init_q;
			s.initElectric = last_init_qe;
			
			if(s.cap < s.initRegular + s.initElectric)
				s.cap = s.initRegular + s.initElectric;
						
			//s.Show(); 
			//printf("sum_init_q: %d sum_init_qe: %d last_init_q: %d last_init_qe: %d local_cntr: %d\n",
			//		sum_init_q, sum_init_qe, last_init_q, last_init_qe, local_cntr);
			//getchar();
        }

        if(s.tgtRegular > s.cap || s.tgtElectric > s.cap)
        {
            printf("Station %d with greater tgt than cap ... adjusting cap ... \n", s.id);
            s.Show();
            s.cap = std::max(s.tgtRegular, s.tgtElectric) + 1;
            modifiedTgt++;
        }
    }
	
	// Do a final loop to remove the ones without targets or init quantities and print output ....
	stations.reserve(3000); 
	for(size_t i=0; i<stations_temp.size(); i++)
		if(stations_temp[i].tgtRegular >= 0 && stations_temp[i].tgtElectric >= 0)
			stations.push_back( stations_temp[i] );
	
	int sum_q =0; int sum_qe = 0;
	for(size_t i=0;i<stations.size();i++)
	{
		sum_q += stations[i].initRegular;
		sum_qe += stations[i].initElectric;
	}

	printf("Loaded %d stations with sum_q: %d sum_qe: %d. ModifiedTgts: %d\n",(int)stations.size(),sum_q,sum_qe,modifiedTgt);
}


#endif