#!/bin/bash

# Determine the build type (default: Release)
BUILD_TYPE="Release"
CXX_FLAGS=""
C_FLAGS=""
USE_CVRPSEP_FLAG=""

# First argument: build type
if [ "$1" == "debug" ]; then
    BUILD_TYPE="Debug"
    CXX_FLAGS="-ggdb3 -fsanitize=undefined -fsanitize=address"
    C_FLAGS="-ggdb3 -fsanitize=undefined -fsanitize=address"
elif [ "$1" == "valgrind" ]; then
    BUILD_TYPE="Debug"
    CXX_FLAGS="-ggdb3"
    C_FLAGS="-ggdb3"
elif [ "$1" == "release" ] || [ -z "$1" ]; then
    BUILD_TYPE="Release"
    CXX_FLAGS="-O3 -march=native -mtune=native -flto=$(nproc) -mavx2 -s"
    C_FLAGS="-O3 -march=native -mtune=native -flto=$(nproc) -mavx2 -s"
fi

echo "Building the executable with configuration: ${BUILD_TYPE}"
echo "CXX_FLAGS=${CXX_FLAGS}"

mkdir -p build
cd build

cmake \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCMAKE_CXX_FLAGS="${CXX_FLAGS}" \
    -DCMAKE_C_FLAGS="${C_FLAGS}" \
    ../src_exact

make -j$(nproc)

