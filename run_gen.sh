#!/bin/bash

cities=("newyork" "chicago" "paris" "montreal" "toronto" "antwerp" "washington" "mexicocity" "madrid" "sanfrancisco" "boston" "barcelona" "rio" "bruxelles" "vancouver" "quebec")

for city in "${cities[@]}"; do
	./gen city_name=$city uncharged_percentage=0.20 seed=19900316
done


