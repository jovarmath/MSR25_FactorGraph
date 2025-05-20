#!/bin/sh

cd 02_trajectory_estimation

DIR="./build/"
if [ -d "$DIR" ]; then
	echo " "
	echo "Running Cmake - make"
	echo " "
	# run cmake
	cd build/
	cmake ..
	make
else 
	mkdir build
	cd build/
	cmake ..
	make
fi

cd ..

#PathDATASET="/mnt/d/trajectory-estimation-data/sbg-data/23_11_15/03/"
#PathCONFIG="/mnt/d/trajectory-estimation-data/sbg-data/23_11_15/03/config/"
#PathOUT="/mnt/d/trajectory-estimation-data/sbg-data/23_11_15/03/02_trajectory/"

cd bin
./main ${PathDATASET} ${PathCONFIG} ${PathOUT}