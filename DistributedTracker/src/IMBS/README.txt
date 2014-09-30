IMBS Background Subtraction Library
-----------------------------------

Introduction
------------ 

This file README.txt contains information about
IMBS algorithm described in
D. D. Bloisi and L. Iocchi
"Independent Multimodal Background Subtraction"
In Proc. of the Third Int. Conf. on Computational Modeling of Objects
Presented in Images: Fundamentals, Methods and Applications, pp. 39-44, 2012.
Please, cite the above paper if you use IMBS.

IMBS software is provided without any warranty about its usability. 
It is for educational purposes and should be regarded as such.

Written by Domenico D. Bloisi

Please, report suggestions/comments/bugs to
domenico.bloisi@gmail.com


Build IMBS2
-----------
IMBS is provided with a CMakeLists.txt file and can be compiled
by using cmake (http://www.cmake.org/).

On windows
1) unzip the file imbs2.zip
2) use the CMake graphical user interface to create the desired
   makefile

On Linux
1) unzip the file imbs2.zip in <imbs2 folder>
2) $cd <imbs2 folder>
3) $mkdir build
4) $cd build
5) $cmake ..
6) $make


Run IMBS2
---------
IMBS is provided with an usage example (main.cpp)

On windows

For video files
>imbs -vid video1.avi

For an image sequence (fps = 25 default value)
>imbs -img images/1.png
or you can specify the fps value
>imbs -img images/1.png -fps 7
Note that the usage example works only on image sequences in which
the filename format is <n>.png, where n is the frame number
(e.g., 7.png).

