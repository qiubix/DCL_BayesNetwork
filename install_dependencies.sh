#!/usr/bin/env sh

# sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y
# sudo apt-get update -qq
# sudo apt-get install libboost-all-dev libopencv-dev libpcl-all

if [ ! -d /tmp/pcl ]; then
  git clone https://github.com/PointCloudLibrary/pcl.git /tmp/pcl
  cd /tmp/pcl && cmake -DCMAKE_INSTALL_PREFIX=/tmp/libpcl .
  #       -DPCL_ONLY_CORE_POINT_TYPES=ON \
    #       -DBUILD_2d=OFF \
    #       -DBUILD_features=OFF \
    #       -DBUILD_filters=OFF \
    #       -DBUILD_geometry=OFF \
    #       -DBUILD_global_tests=OFF \
    #       -DBUILD_io=ON \
    #       -DBUILD_kdtree=OFF \
    #       -DBUILD_keypoints=OFF \
    #       -DBUILD_ml=OFF \
    #       -DBUILD_octree=ON \
    #       -DBUILD_outofcore=OFF \
    #       -DBUILD_people=OFF \
    #       -DBUILD_recognition=OFF \
    #       -DBUILD_registration=OFF \
    #       -DBUILD_sample_consensus=OFF \
    #       -DBUILD_search=OFF \
    #       -DBUILD_segmentation=OFF \
    #       -DBUILD_stereo=OFF \
    #       -DBUILD_surface=OFF \
    #       -DBUILD_tools=OFF \
    #       -DBUILD_tracking=OFF \
    #       -DBUILD_visualization=OFF .
  make pcl_common/fast pcl_octree/fast pcl_io -j2
fi
