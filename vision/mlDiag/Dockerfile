FROM ubuntu:20.04
MAINTAINER docker@andrepool.com

ENV DEBIAN_FRONTEND noninteractive

# fetch the ubuntu pacakges from local mirror (mirrors are not available for security.ubuntu.com)
RUN sed -i -e 's/archive\.ubuntu\.com/nl\.archive\.ubuntu\.com/' /etc/apt/sources.list

RUN apt-get update
RUN apt-get upgrade -y

# debconf: delaying package configuration, since apt-utils is not installed
RUN apt-get -y -q install apt-utils

# TODO: find out if there is a subset of qt5 that would be enough (the next line probably adds about 800MiB)
RUN apt-get -y install qt5-default
# RUN apt-get -y install qt5-qmake 

RUN apt-get -y install nvidia-driver-455 # very large, do we realy need al that stuff?

RUN apt-get -y install freeglut3-dev g++ libglu1-mesa-dev make mesa-common-dev net-tools

RUN mkdir /tmp/runtime-root # for XDG_RUNTIME_DIR

WORKDIR /project

RUN mkdir build

COPY *.pro *.ui ./*.cpp ./*.hpp ./
RUN qmake -o build/Makefile mlDiag.pro

WORKDIR /project/build

RUN make -j 8

CMD ["./mlDiag"]


