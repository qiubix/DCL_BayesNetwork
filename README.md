DCL_BayesNetwork
================

[![Build Status](https://travis-ci.org/qiubix/DCL_BayesNetwork.svg?branch=master)](https://travis-ci.org/qiubix/DCL_BayesNetwork)

Description
-----------
DisCODe components for creating bayesian network and conducting reasoning based on this network.

##Dependencies
SMILE library:
http://genie.sis.pitt.edu/index.php/downloads

PCL library:
http://pointclouds.org/

OpenCV library:
http://opencv.org/

### Another DCL dependencies:
This DCL depends on couple other DisCODe libraries, so they have to be present in `DISCODE_DCL_DIR` before building. Currently it depends on the following ones, but in the future releases it will surely become more independent.
* CvBasic
* PCL
* PCLCoreTypes
* SIFTObjectModel

## Installation
Repository contains script `install_discode.sh`, which checks all dependencies and installs them, if necessary.
After installing all dependencies just clone this repository to `DISCODE_DCL_DIR` and then run:

`mkdir build && cd build && cmake .. && make -j5 && make install`
