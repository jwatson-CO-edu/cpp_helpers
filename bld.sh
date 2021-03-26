#!/bin/bash
chmod u+x bld.sh

# We don't typically need to erase `build` from a previous cmake run unless it gets confused
if false; then 
    rm -rf build
fi

# Create the 'build' directory if it does not exist
if [ ! -d "build" ]; then
    echo "./build missing, create ..."
    mkdir build # Create the build directory
else
    echo "Found ./build ..."
fi

if [ ! -d "lib" ]; then
    echo "./lib missing, create ..."
    mkdir lib # Create the build directory
else
    echo "Found ./lib ..."
fi

if [ ! -d "bin" ]; then
    echo "./bin missing, create ..."
    mkdir bin # Create the build directory
else
    echo "Found ./bin ..."
fi

if [ ! -d "include" ]; then
    echo "./include missing, create ..."
    mkdir include # Create the build directory
else
    echo "Found ./include ..."
fi

echo "Ready to compile!"
echo

cd build # Change PWD to tbe build directory
cmake .. # Ask CMake to AUTOMATICALLY generate scripts that will build the entire project
make # Invoke `make`, notice that we did not have to write a Makefile
cd .. # Change PWD back to the project directory

echo "Build script finished!"
echo