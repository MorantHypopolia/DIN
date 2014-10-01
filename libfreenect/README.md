libfreenect
===========

# Requirements

libfreenect requires the following packages to build:

  * build-essential
  * g++
  * cmake
  * libusb

On Xubuntu (Ubuntu) 12.04.4 LTS (Kernel 3.2.0-68) these dependencies are resolved by installing the
following packages:

  - build-essential
  - cmake
  - libusb-dev
  - libusb-1.0-0-dev

# How to build

The only development platform is Linux. We recommend a so-called out of source
build which can be achieved by the following command sequence:

  - mkdir build
  - cd build
  - cmake ..
  - make -j\<number-of-cores+1\>

# Installation

Once the build phase has been successfully, the library have to be installed
so that it can be included in other projects. This is achieved by the
following command sequence:

  - cd build
  - sudo make install
