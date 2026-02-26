#!/bin/bash

### Installation guide from http://ceres-solver.org/installation.html

# CMake
sudo apt-get install cmake ament-cmake -y
# gflags (DO NOT INSTALL libgoogle-glog-dev as it can potentially break your packages or your system on ubuntu 22.04 and above)
sudo apt-get install libgflags-dev -y
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev -y
# Eigen3
sudo apt-get install libeigen3-dev -y 
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev -y 

###

cd ~
wait

mkdir thirdparty
wait

#install ceres solver
cd ~/thirdparty/
wait

git clone --recursive https://github.com/ceres-solver/ceres-solver.git
wait

cd ceres-solver
wait

mkdir build
wait

cd build
wait

cmake ..

wait

make -j $(nproc)
wait

make test
wait

sudo make install -j $(nproc)