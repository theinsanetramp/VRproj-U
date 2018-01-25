# Locate potrace
# This module defines
# POTRACE_LIBRARY
# POTRACE_FOUND, if false, do not try to link to potrace 
# POTRACE_INCLUDE_DIR, where to find the headers
#
# $POTRACE_DIR is an environment variable that would
# correspond to the ./configure --prefix=$POTRACE_DIR
#
# Created by Borja Sanchez Zamorano 

FIND_PATH(POTRACE_INCLUDE_DIR potracelib.h
	C:/workspace/workspace-HEADRASTER/potrace-1.8/src
	/home/borsanza/nacho/WRaster/potrace/src
)

FIND_LIBRARY(POTRACE_LIBRARY 
	NAMES libpotrace180 potrace POTRACE
	PATHS
	C:/workspace/workspace-HEADRASTER/binaries/w32
	/home/borsanza/nacho/WRaster/binaries/linux
)

SET(POTRACE_FOUND "NO")
IF(POTRACE_LIBRARY AND POTRACE_INCLUDE_DIR)
	SET(POTRACE_FOUND "YES")
ENDIF(POTRACE_LIBRARY AND POTRACE_INCLUDE_DIR)

IF(POTRACE_LIBRARY)
	MESSAGE(STATUS "POTRACE_LIBRARY: " ${POTRACE_LIBRARY})
ELSE(POTRACE_LIBRARY)
	MESSAGE(STATUS "POTRACE_LIBRARY: NOT FOUND")
ENDIF(POTRACE_LIBRARY)

IF(POTRACE_INCLUDE_DIR)
	MESSAGE(STATUS "POTRACE_INCLUDE_DIR: " ${POTRACE_INCLUDE_DIR})
ELSE(POTRACE_INCLUDE_DIR)
	MESSAGE(STATUS "POTRACE_INCLUDE_DIR: NOT FOUND")
ENDIF(POTRACE_INCLUDE_DIR)
