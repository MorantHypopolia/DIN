# - Try to find Freenect include dirs and libraries
#	
#	Usage of this module as follows:
#	
#	  find_package(Freenect)
#	
#	Variables used by this module, they can change the default behaviour and need
#	to be set before calling find_package:
#	
#	  Freenect_ROOT_DIR			Set this variable to the root installation of
#								Freenect if the module has problems finding the
#								proper installation path.
#	
#	Variables defined by this module:
#	
#	  Freenect_FOUND			System has Freenect, include and library dirs found
#	  Freenect_INCLUDE_DIR		The Freenect include directories.
#	  Freenect_LIBRARY			The Freenect library

find_path(Freenect_ROOT_DIR NAMES libfreenect.hpp)
set(Freenect_INCLUDE_DIR ${Freenect_ROOT_DIR}/libfreenect)

find_library(Freenect_LIBRARY NAMES freenect HINTS ${Freenect_ROOT_DIR}/../lib64)

if (Freenect_LIBRARY STREQUAL "Freenect_LIBRARY-NOTFOUND")
	if (Freenect_FIND_REQUIRED)
		message(FATAL_ERROR "Freenect library has NOT been found. Did you install the library?.")
	endif (Freenect_FIND_REQUIRED)
	
	set(Freenect_FOUND 0)
else (Freenect_LIBRARY STREQUAL "Freenect_LIBRARY-NOTFOUND")
	message(STATUS "Found Freenect")
	set(Freenect_FOUND 1)
endif (Freenect_LIBRARY STREQUAL "Freenect_LIBRARY-NOTFOUND")
