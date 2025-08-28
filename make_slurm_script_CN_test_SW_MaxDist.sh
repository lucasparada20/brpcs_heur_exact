#!/bin/bash

# Define directories
INSTANCE_DIR=~/work/brpcs/instances
RESULTS_DIR=~/work/brpcs/results
SCRIPTS_DIR=~/work/brpcs/scripts
RUN_SCRIPT=run_slurm_scripts.sh

CONSTRUCTION_HEURISTIC=("SEQ" "REG2" "REG3" "ALL")
MAX_DISTANCES=("100" "90" "80" "70" "60" "50")
CITIES_TO_LAUNCH=("antwerp" "barcelona" "boston" "bruxelles" "chicago" "madrid" "newyork" "paris" "toronto")

echo "To launch cities:${#CITIES_TO_LAUNCH[@]} heuristics:${#CONSTRUCTION_HEURISTIC[@]} distances:${#MAX_DISTANCES[@]}."
echo "Should be $((${#CITIES_TO_LAUNCH[@]} * ${#CONSTRUCTION_HEURISTIC[@]} * ${#MAX_DISTANCES[@]})) scripts in total."

# Ensure the scripts directory exists
mkdir -p "$SCRIPTS_DIR"

script_count=0

#  Loop though all construction heuristics
for dist in "${MAX_DISTANCES[@]}"; do
	#  Loop though all construction heuristics
	for heur in "${CONSTRUCTION_HEURISTIC[@]}"; do
		# Loop through each file in the instance directory
		for file in "$INSTANCE_DIR"/*_SW.txt; do
			if [[ -f "$file" ]]; then
				filename=$(basename "$file")  # Converts : path/to/file.txt into file.txt
				base_name="${filename%.txt}"  # Removes .txt extension
				
				# sed: remove everything after first underscore, then strip digits
				city=$(echo "$base_name" | sed 's/_.*//; s/[0-9].*//')
				
				found=false
				
				for c in "${CITIES_TO_LAUNCH[@]}"; do
					if [ "$city" == "$c" ]; then
						found=true
						break
					fi
				done
				
				if [ "$found" == false ]; then
					continue
				fi
				
				#echo ${base_name} $part $city
				
				re_file="re_CN_${heur}_${dist}_${city}_20_SW.txt" # Expected result file
				
				# Check if the corresponding result file exists
				if [[ -f "$RESULTS_DIR/$re_file" ]]; then
					echo "Skipping $filename since $re_file exists in results."
					continue
				else 
					echo "${re_file} for $city does not exist in results dir. Creating the script ..."
				fi

				script_name="$SCRIPTS_DIR/${heur}_${dist}_${base_name}.sh"

				# Create the script with SLURM directives
				cat > "$script_name" <<EOF
#!/bin/bash -l
#SBATCH --mem=4G
#SBATCH --time=14:00:00
#SBATCH --account=def-cotej
#SBATCH --cpus-per-task=1

build/exec_heur instance_file=$file construction_heuristic=$heur max_route_distance=$dist > output/${heur}_${dist}_${base_name}.out 2> output/${heur}_${dist}_${base_name}.err
EOF

				chmod +x "$script_name"
				echo "Created script: $script_name"
				((script_count++))  # Increment script count
			fi
		done
	done
done





echo "Total scripts created: $script_count"

# Create `run_slurm_scripts.sh` in the current directory
RUN_SCRIPT_PATH="./$RUN_SCRIPT"
echo "#!/bin/bash" > "$RUN_SCRIPT_PATH"

for script in "$SCRIPTS_DIR"/*.sh; do
    if [[ -f "$script" ]]; then
        echo "sbatch $script" >> "$RUN_SCRIPT_PATH"
    fi
done

chmod +x "$RUN_SCRIPT_PATH"

echo "Created SLURM batch submission script: $RUN_SCRIPT_PATH"
echo "Script generation complete."
