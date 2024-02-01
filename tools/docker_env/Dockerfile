FROM ubuntu:20.04
MAINTAINER QingfengLi liqingfengzzu@163.com
ARG DEBIAN_FRONTEND=noninteractive
COPY sources.list /etc/apt/sources.list
RUN apt-get update && apt install -y curl\ 
    && apt install -y libeigen3-dev libboost-all-dev \
    && apt install -y cmake
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py \
    && python3 get-pip.py \
    && rm get-pip.py \
    && pip install cpplint

