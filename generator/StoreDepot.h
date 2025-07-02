#ifndef STORE_DEPOT_H
#define STORE_DEPOT_H

#include "Station.h"
#include <vector>
#include <string>

void StoreDepot(std::string & city_name, std::vector<Station> & stations)
{
	Station depot; depot.name = "depot"; depot.id = 0; depot.cap = 0;

	//HQ of Empresa Municipal de Transportes de Madrid, operator of BiciMad
	if(city_name == "madrid")
	{
		depot.lat = 40.39705; depot.lon = -3.67147;
	}	
	// Bixi's headquarters are defined at one of its' warehouses in: 5945 Av. de Gaspé
	//45.5298971, -73.6017523	
	if(city_name == "montreal")
	{
		depot.lon = -73.6017523; depot.lat = 45.5298971;
	}	
	// àVelo Quebec headquarters are defined at le siège social de RTC, the company that manages the service (in conjunction with PBSC solutions)
	//46.83776819782474, -71.27563194775138
	if(city_name == "quebec")
	{
		depot.lon = -71.27563194775138; depot.lat = 46.83776819782474;
	}	
	// Bike Itaú headquarters are defined at mid Copacabana. Nonetheless the service is managed by the company TemRio whose headquarters are in São Paulo.
	// A Tembici é a empresa que gere e mantém o sistema Bike Itaú no Rio de Janeiro, desde 2018. 
	//-22.96941223958878, -43.18651734748761 (downtown Copa)
	if(city_name == "rio")
	{
		depot.lon = -43.18651734748761; depot.lat = -22.96941223958878;
	}
	//HQ of Toronto Parking Authority, operator of Toronto Bike Share
	if(city_name == "toronto")
	{
		depot.lat = 43.65282; depot.lon = -79.37729;
	}
	
	//In Antwerp, Belgium, Donkey Republic, a Danish company, operates the shared electric bicycle system, while Lantis, the Antwerp contracting authority, is responsible for managing the deployment of the bikes. Donkey Republic provides and operates the bikes, while Lantis manages the project on behalf of the Antwerp region.
	if(city_name == "antwerp")
	{
		//Lantis coordinates : 51.227107370122326, 4.400347855652784
		depot.lat = 51.22710 ; depot.lon = 4.4003478;
	}
	if(city_name == "washington")
	{
		// Capital Bikeshare's headquarters are located at 1501 Wilson Blvd Ste 1100, Arlington, Virginia, 22209
		//38.89538947941347, -77.07424659999998	
		depot.lon = -77.0742465; depot.lat = 38.895389;
	}	
	if(city_name == "sanfrancisco")
	{
		//Metropolitan Transportation Commission in SF.
		depot.lat = 37.78826; depot.lon = -122.39155;
	}
	if(city_name == "boston")
	{
		// Blue Bikes headquarters defined at 100 Northern Avenue, Boston, MA 02210
		//42.3517° N, 71.0405° W. 
		depot.lon = -71.0405; depot.lat = 42.3517; 
	}
	if(city_name == "lyon")
	{
		//Le siège social de Vélo'v, le système de vélos en libre-service de Lyon, est situé à Lyon. Plus précisément, il se trouve au 7020 rue Gerland, 69007 Lyon.
		depot.lat = 45.73488; depot.lon = 4.83954;
	}
	if(city_name == "chicago")
	{
		// Divvy Service Warehouse : 41.89695600489008, -87.67771845224394
		depot.lat = 41.89695600489008; depot.lon = -87.67771845224394;
	}
	if(city_name == "paris")
	{
		//The Velib' headquarters is located at 4 place de la Pyramide, 92919 Puteaux. This is the registered office of SMOVENGO, the company that publishes the Velib website.
		depot.lat = 48.888534; depot.lon = 2.242429;
	}
	if(city_name == "newyork")
	{
		// Citi Bike's headquarters are located at Brooklyn, 3A 220 36th St, United States
		//40.65686421320091, -74.00812771723888	
		depot.lon = -74.00812771723888; depot.lat = 40.65686421320091; 
	}
	if(city_name == "mexicocity")
	{
		//Oficinas ecobici mx df
		depot.lat = 19.42997; depot.lon = -99.17846;	
	}
	if(city_name == "barcelona")
	{
		// Bicing Barcelona headquarters are defined at Pedalem Barcelona, the company that manages the service
		//41.34799668529817, 2.1197057380905058
		depot.lon = 2.1197057380905058; depot.lat = 41.34799668529817;		
	}
	if(city_name == "bruxelles")
	{
		depot.lat = 50.841478; depot.lon = 4.3525995;
	}
	if(city_name == "saopaulo")
	{
		// A sede da empresa está localizada na Avenida Paulista, nº 2.439, 5º andar, Bela Vista, São Paulo – SP
		depot.lat = -23.564657; depot.lon = -46.642753;
	}
	if(city_name == "santiago")
	{
		// La empresa Tembici, que opera el sistema de bicicletas compartidas Bikesantiago en Santiago de Chile, tiene su dirección principal en Av. Eduardo Frei Montalva N° 3020, comuna de Renca, Región Metropolitana.
		depot.lat = -33.402142; depot.lon = -70.682880;
	}
	
	stations.push_back(depot);
}

#endif