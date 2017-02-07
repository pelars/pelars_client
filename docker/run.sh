#!/bin/bash

for var in "$@"
do
    VARS=$VARS" "$var
done

export docker_path=`pwd`

xhost +

#FOLDERS="-v ${pelars_path}/sensor_manager_top:/sensor_manger_top"
NVIDIA="--device /dev/nvidia0 --device /dev/nvidiactl --device /dev/nvidia-uvm"
KINECT2="-v /etc/udev/rules.d:/etc/udev/rules.d $(${docker_path}/mapkinect.sh)"
VIDEO="-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
OPT="--net=host -t -i --privileged"
NAME="pelars_build"
docker run $OPT $NVIDIA $KINECT2 $VIDEO $NAME /bin/bash -c "$VARS"


#TO COMMIT RUNNING CONTAINER (docker ps -l for container id)
#docker commit -m="MESSAGE" ID_MACCHINA NOME_COMMIT
#si riesegue con docker run NOME_COMMIT