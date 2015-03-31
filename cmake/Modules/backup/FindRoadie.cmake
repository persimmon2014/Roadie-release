# Try to find libRoadie library
# Once done this will define
#  LIBROADIE_FOUND - if system found libRoadie library
#  LIBROADIE_INCLUDE_DIRS - The libRoadie include directories
#  LIBROADIE_LIBRARIES - The libraries needed to use libRoadie
#  LIBROADIE_DEFINITIONS - Compiler switches required for using libRoadie


set (LIBROADIE_ROOT_DIR ${CMAKE_SOURCE_DIR}/../Roadie/)
message(STATUS "LIBROADIE_ROOT_DIR: " ${LIBROADIE_ROOT_DIR})

find_path(LIBROADIE_INCLUDE_DIR
	NAMES hwm_network.hpp
	PATHS ${LIBROADIE_ROOT_DIR}/include
	DOC "The libRoadie include directory"
)

find_library(LIBROADIE_LIBRARY
	NAMES libRoadie.so
	PATHS ${LIBROADIE_ROOT_DIR}/lib
	DOC "The libRoadie library"
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LOGGING_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LIBROADIE DEFAULT_MSG LIBROADIE_INCLUDE_DIR LIBROADIE_LIBRARY)

if (LIBROADIE_FOUND)
    set(LIBROADIE_LIBRARIES ${LIBROADIE_LIBRARY} )
    set(LIBROADIE_INCLUDE_DIRS ${LIBROADIE_INCLUDE_DIR} )
    set(LIBROADIE_DEFINITIONS )
endif(LIBROADIE_FOUND)

# Tell cmake GUIs to ignore the "local" variables.
mark_as_advanced(LIBROADIE_ROOT_DIR LIBROADIE_INCLUDE_DIR LIBROADIE_LIBRARY)
