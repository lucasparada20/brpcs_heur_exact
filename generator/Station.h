#ifndef STATION_H
#define STATION_H

#include <iostream>
#include <string>

struct Station {
    Station() 
        : id(-1), cap(-1), initRegular(-1), tgtRegular(-1), 
          initElectric(-1), uElectric(-1), tgtElectric(-1), charges(-1), 
          bss_station_id(""), name(""), lat(0.0), lon(0.0) {}

	// Constructor that forwards arguments for emplace_back() in station_information
    Station(int id, int cap, int charges, const std::string& bss_station_id, const std::string& name, 
            double lat, double lon)
        : id(id), cap(cap), charges(charges), bss_station_id(bss_station_id), name(name), lat(lat), lon(lon) {
			initRegular = -1; tgtRegular = -1; initElectric = -1; tgtElectric = -1; uElectric = -1;
		}
	// Constructor that forwards arguments for emplace_back() in station_status
	Station(int initRegular, int initElectric, std::string & bss_station_id) : initRegular(initRegular), initElectric(initElectric), bss_station_id(bss_station_id)  {
		id = -1; tgtRegular = -1; cap = -1; tgtElectric = -1; uElectric = -1; charges = 0; name = ""; lat = 0.0; lon = 0.0;
	}
	
	Station & operator= ( const Station & other )
	{
		//std::cout << "Assigning Station: id=" << other.id
        //          << " bss_station_id='" << other.bss_station_id << "'"
        //          << " name='" << other.name << "'" << std::endl;
		
		if (this != &other)
		{
			id = other.id; 
			cap = other.cap;
			initRegular = other.initRegular;
			tgtRegular = other.tgtRegular;
			initElectric = other.initElectric;
			uElectric = other.uElectric;
			tgtElectric = other.tgtElectric;
			charges = other.charges;
			bss_station_id = other.bss_station_id;
			name = other.name;
			lat = other.lat;
			lon = other.lon;
		}		
		return *this;
	}
	
    int id; 
    int cap;
    int initRegular;
    int tgtRegular;
    int initElectric;
	int uElectric;
    int tgtElectric;
    int charges;
    std::string bss_station_id;
    std::string name;
    double lat;
    double lon;

    void Show() const {
        std::cout << "Station id:" << id 
                  << " charges:" << charges 
                  << " initReg:" << initRegular 
                  << " tgtReg:" << tgtRegular 
                  << " initElec:" << initElectric
				  << " uElec:" << uElectric
                  << " tgtElec:" << tgtElectric 
				  << " bss_station_id:" << bss_station_id
                  //<< " lat:" << lat << " lon:" << lon 
                  << " cap:" << cap << " name:" << name 
                  << std::endl;
    }
};

#endif
