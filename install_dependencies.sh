#!/usr/bin/env sh

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y
sudo apt-get update -qq
sudo apt-get install libboost-all-dev libopencv-dev libpcl-all
