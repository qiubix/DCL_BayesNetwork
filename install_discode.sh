#!/usr/bin/env sh

mkdir -p ~/src
if [ ! -d ~/src/DisCODe ]; then
  git clone https://github.com/maciek-slon/DisCODe.git ~/src/DisCODe
  cd ~/src/DisCODe && mkdir build && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=`pwd`/inst
  make -j3
  make install
fi

mkdir -p ~/src/DCL
# if [ ! -d ~/src/DCL/CvCoreTypes ]; then
#   git clone https://github.com/DisCODe/CvCoreTypes.git ~/src/DCL/CvCoreTypes
#   cd ~/src/DCL/CvCoreTypes && mkdir build && cd build
#   cmake .. && make -j3 && make install
# fi

# if [ ! -d ~/src/DCL/CvBasic ]; then
#   git clone https://github.com/DisCODe/DCL_CvBasic ~/src/DCL/CvBasic
#   cd ~/src/DCL/CvBasic && mkdir build && cd build
#   cmake .. -DCMAKE_SKIP_INSTALL_ALL_DEPENDENCY=true && make Sequence CvSIFT -j3 && make install/fast
# fi

# if [ ! -d ~/src/DCL/PCLCoreTypes ]; then
#   git clone https://github.com/DisCODe/PCLCoreTypes ~/src/DCL/PCLCoreTypes
#   cd ~/src/DCL/PCLCoreTypes && mkdir build && cd build
#   cmake .. -DCMAKE_SKIP_INSTALL_ALL_DEPENDENCY=true && make -j3 && make install
# fi

# if [ ! -d ~/src/DCL/PCL ]; then
#   git clone https://github.com/DisCODe/PCL ~/src/DCL/PCL
#   cd ~/src/DCL/PCL && mkdir build && cd build
#   cmake .. -DCMAKE_SKIP_INSTALL_ALL_DEPENDENCY=true && make PCDReader -j3 && make install/fast
# fi

# if [ ! -d ~/src/DCL/SIFTObjectModel ]; then
#   git clone https://github.com/DisCODe/SIFTObjectModel ~/src/DCL/SIFTObjectModel
#   cd ~/src/DCL/SIFTObjectModel && mkdir build && cd build
#   cmake .. -DCMAKE_SKIP_INSTALL_ALL_DEPENDENCY=true && make SOMJSONReader FeatureCloudConverter SIFTAdder -j3 && make install/fast
# fi
