#!/bin/bash

EXEC_FILE="build/exec_heur"

if [[ -x "$EXEC_FILE" ]]; then
    "$EXEC_FILE" instance_file=instances/quebec167_20_SW.txt
else
    echo "Error: Executable '$EXEC_FILE' not found or not executable. Did you build it first ... ?"
    exit 1
fi
