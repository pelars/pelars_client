FROM ubuntu:14.04

CUDA_FILE=cuda_7.5.18_linux.run
USER=giacomo

RUN apt-get update && apt-get install -q -y wget build-essential autotools-dev automake

RUN mkdir /nvidia
ADD . /nvidia/

RUN chmod +x /nvidia/${CUDA_FILE}
RUN /nvidia/${CUDA_FILE} --toolkit --silent
RUN rm /nvidia/${CUDA_FILE} 

#BUILD DEPENDENCY

RUN apt-get install -y ncurses-dev cmake cmake-curses-gui git nano
RUN apt-get install -y python-dev libtool
RUN apt-get install -y libbz2-dev nasm libcairo2-dev
RUN apt-get install -y libssl-dev libssh2-1-dev openssl
RUN apt-get install -y freeglut3 freeglut3-dev 
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libxmu-dev libxi-dev libqrencode-dev
RUN apt-get install -y libyaml-cpp-dev 

#RUNTIME DEPENDENCY

RUN apt-get install -y terminator

#ADD LIBRARIES

RUN mkdir /libraries
ADD /home/${USER}/libraries/ /libraries/