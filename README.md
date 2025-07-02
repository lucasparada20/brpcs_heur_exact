# brpcs_heur
An implementation of the ALNS meta heuristic for the bicycle repositioning problem with charging stations

## Building the code in Linux

1. Clone the repository and add executable permission to a script that will call CMake for you:

```shell
git clone https://github.com/lucasparada20/brpcs_heur.git
cd brpcs_heur
chmod u+x cmake_script_heur.sh
```
2. Build the code by typing:

```bash
./cmake_script_heur.sh
```

if you want to debug or use valgrind, just type:

```bash
./cmake_script_heur.sh debug
```

```bash
./cmake_script_heur.sh valgrind
```

Alternatively, inside the directory `src_heur`, you will also find a `Makefile` that can build the code by typing:

```bash
(cd src_heur && make) # Assuming you are in the brpcs_heur directory
```

To call the executable for these instances, the script `run_heur.sh` contains all the shell commands you need, for example:

```bash
build/exec_heur instance_file=instances/quebec166_20.txt
```

Lastly, this code will store results and output files in a directory named `results`. So, be sure to create it inside the `brpcs_heur` directory first.
