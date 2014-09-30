cmake_minimum_required(VERSION 2.8)

project(DistributedTracker)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

# Build type configuration
if (CMAKE_BUILD_TYPE)
	string(TOUPPER ${CMAKE_BUILD_TYPE} _build_type)
	set(CMAKE_BUILD_TYPE ${_build_type} CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE)

# Build type must be either DEBUG or RELEASE
if (CMAKE_BUILD_TYPE STREQUAL "")
	message("No build type specified: forcing to RELEASE")
	set(CMAKE_BUILD_TYPE "RELEASE" CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE STREQUAL "")

add_definitions(-Wall)

if (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	add_definitions(-ggdb)
elseif (CMAKE_BUILD_TYPE STREQUAL "RELEASE")
	add_definitions(-O3)
else (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	message("Invalid build type specified: doing nothing")
endif (CMAKE_BUILD_TYPE STREQUAL "DEBUG")

set(EXECUTABLE_OUTPUT_PATH ../bin)

find_package(Boost REQUIRED)
find_package(OpenCV 2.4.2 REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(Freenect REQUIRED)
find_package(PTracking REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${PTracking_INCLUDE_DIR})

file(GLOB_RECURSE CameraModel_src "CameraModel/*.cpp")
file(GLOB_RECURSE Imbs_src "IMBS/*.cpp")
file(GLOB_RECURSE KinectDataAcquisition_src "KinectDataAcquisition/*.cpp")
file(GLOB_RECURSE Utils_src "Utils/*.cpp")

add_executable(DistributedTracker main.cpp "${PTracking_INCLUDE_DIR}/PTracker/PTracker.cpp" ${CameraModel_src} ${Imbs_src} ${KinectDataAcquisition_src} ${Utils_src})
target_link_libraries(DistributedTracker ${OpenCV_LIBS} ${LIBXML2_LIBRARIES} ${PTracking_LIBRARY} freenect pthread boost_system boost_thread)
