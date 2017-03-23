#!/bin/bash

bad=$(fuser /dev/video0)
for i in $bad; do
	echo "killing $i which was using the webcam"
	kill -9 $i
done

for var in "$@"
do
    VARS=$VARS" "$var
done

xhost +
export working_path="/home/lando/pelars/client/pelars_clientdeploy/docker"
cd ..
export pelars_path="/home/lando/pelars/client/pelars_clientdeploy"
cd data
export data_path="/home/lando/pelars/client/pelars_clientdeploy/data"
cd $working_path

cd /usr/local/cuda-7.5/samples/0_Simple/matrixMul
./matrixMul >> /dev/null

cd $working_path

#--user=${USER}
#-v /dev/:/dev/
#-v /etc/sudoers.d:/etc/sudoers.d:ro -v /etc/shadow:/etc/shadow:ro -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro

docker run --rm -t -i --privileged --net=host -v /dev/snd:/dev/snd --device=/dev/video0:/dev/video0  -v /dev/bus/usb:/dev/bus/usb -v /etc/udev/rules.d:/etc/udev/rules.d --device=/dev/nvidia0:/dev/nvidia0 --device=/dev/nvidiactl:/dev/nvidiactl --device=/dev/nvidia-uvm:/dev/nvidia-uvm -e LD_LIBRARY_PATH='/usr/local/lib' -v $data_path:/data -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw pelars_compile:latest /bin/bash  #/pelars/bin/sensors/sensor_manager $VARS

#if not working internet add --net=host
