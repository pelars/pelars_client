#!/bin/bash

CUDA_FILE=cuda_7.5.18_linux.run
#DRIVER_VERSION=361.93.02
#DRIVER_FILE=NVIDIA-Linux-x86_64-${DRIVER_VERSION}.run 

wget http://developer.download.nvidia.com/compute/cuda/7.5/Prod/local_installers/${CUDA_FILE}
#wget http://it.download.nvidia.com/XFree86/Linux-x86_64/${DRIVER_VERSION}/${DRIVER_FILE}

sudo service docker start
docker build -t pelars_build .

