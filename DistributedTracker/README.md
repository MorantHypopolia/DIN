DistributedTracker
=========

# Requirements

DistributedTracker requires the following packages to build:

  * build-essential
  * g++
  * cmake
  * libxml2
  * opencv
  * libfreenect (https://github.com/fabioprev/libfreenect.git - a request for getting the access is required: previtali@dis.uniroma1.it)
  * ptracking (https://github.com/fabioprev/ptracking.git - a request for getting the access is required: previtali@dis.uniroma1.it)

On Xubuntu (Ubuntu) 12.04.4 LTS (Kernel 3.2.0-68) these dependencies are resolved by installing the
following packages:

  - build-essential
  - cmake
  - libxml2
  - libxml2-dev
  - libopencv-dev

and building the following ones:
  
  - libfreenect
  - ptracking

# How to build

The only development platform is Linux. We recommend a so-called out of source
build which can be achieved by the following command sequence:

  - mkdir build
  - cd build
  - cmake ../src
  - make -j<number-of-cores+1>

Note: The DistributedTracker package requires an OpenCV version >= 2.4.2. Therefore, if the
requirements are not satisfied under your Linux distribution, you have to:

  - download OpenCV (http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip/download)
  - compile and install the OpenCV library, following the tutorial

# Usage

To execute DistributedTracker just type in to a terminal the following command sequence:
  
  - cd scripts
  - ./run <agent-number>
