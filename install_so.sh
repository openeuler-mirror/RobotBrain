#!/bin/bash
cp -r ./include/orocos /usr/local/include
cp -r ./include/robot_brain /usr/local/include
cp ./lib/*.so /usr/local/lib
cp -r ./lib/orocos /usr/local/lib
ldconfig