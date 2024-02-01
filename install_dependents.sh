#!/bin/bash
###
# 该脚本用于检查并安装相关依赖文件，方便库的编译工作
###
system_os=`cat /etc/os-release | grep ^ID= |awk -F'=' '{print $2}' | xargs | head -1`
echo 'The System is '$system_os
if [ $system_os = 'ubuntu' ]; then
  echo "=================安装依赖库和工具================="
  # 安装开发工具
  apt install -y git g++-10
  # 代码编译依赖的包
  apt install -y libeigen3-dev libboost-all-dev
  # apt install -y protobuf-compiler libprotobuf-dev
  # 代码开发常用的一些依赖包
  apt install -y clang-format clangd-12 gdb
  update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 100
  #  代码格式检查工具
  apt install -y python3-pip
  pip3 install cpplint
fi
