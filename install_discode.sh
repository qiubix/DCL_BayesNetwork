#!/usr/bin/env sh

echo $(pwd)
mkdir -p ~/src
git clone https://github.com/DisCODe/DisCODe.git ~/src/DisCODe
cd ~/src/DisCODe && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=`pwd`/inst
make -j5
make install
echo $(pwd)

mkdir -p ~/src/DCL
git clone https://github.com/DisCODe/CvCoreTypes.git ~/src/DCL/CvCoreTypes
cd ~/src/DCL/CvCoreTypes && mkdir build && cd build
cmake .. && make -j5 && make install

git clone https://github.com/DisCODe/DCL_CvBasic ~/src/DCL/CvBasic
cd ~/src/DCL/CvBasic && mkdir build && cd build
cmake .. && make -j5 && make install

git clone https://github.com/DisCODe/PCLCoreTypes ~/src/DCL/PCLCoreTypes
cd ~/src/DCL/PCLCoreTypes && mkdir build && cd build
cmake .. && make -j5 && make install

git clone https://github.com/DisCODe/PCL ~/src/DCL/PCL
cd ~/src/DCL/PCL && mkdir build && cd build
cmake .. && make -j5 && make install
