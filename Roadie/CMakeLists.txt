cmake_minimum_required(VERSION 2.8)
project(Roadie)

# Turn on cairo option in the code.
add_definitions(-DHAVE_CAIRO)

# Find packages
find_package(GLIBMM2 REQUIRED)
if(NOT GLIBMM2_FOUND)
message(FATAL_ERROR "Package Glibmm2 not found")
endif(NOT GLIBMM2_FOUND)
include_directories(${GLIBMM2_INCLUDE_DIR})

find_package(GLIB REQUIRED)
if(NOT GLIB_FOUND)
message(FATAL_ERROR "Package GLIB not found")
endif(NOT GLIB_FOUND)
include_directories(${GLIB_INCLUDE_DIRS})

find_package(Cairo REQUIRED)
if(NOT CAIRO_FOUND)
message(FATAL_ERROR "Package Cairo not found")
endif(NOT CAIRO_FOUND)
include_directories(${CAIRO_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)
if(NOT OPENGL_FOUND)
message(FATAL_ERROR "Package OpenGL not found")
endif(NOT OPENGL_FOUND)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLEW REQUIRED)
if(NOT GLEW_FOUND)
message(FATAL_ERROR "Package GLEW not found")
endif(NOT GLEW_FOUND)
include_directories(${GLEW_INCLUDE_DIRS})

find_package(Tvmet REQUIRED)
if(NOT TVMET_FOUND)
message(FATAL_ERROR "Package Tvmet not found")
endif(NOT TVMET_FOUND)
include_directories(${TVMET_INCLUDE_DIRS})

find_package(LibXML++ REQUIRED)
if(NOT LibXML++_FOUND)
message(FATAL_ERROR "Package LibXML++ not found")
endif(NOT LibXML++_FOUND)
include_directories(${LibXML++_INCLUDE_DIRS})

# Specify header files
set(ROADIE_HEADERS
  include/arc_road.hpp
  include/functions.hpp
  include/geometric.hpp
  include/hilbert.hpp
  include/hwm_draw.hpp
  include/hwm_network.hpp
  include/hwm_texture_gen.hpp
  include/im_heightfield.hpp
  include/libroad_common.hpp
  include/osm_network.hpp
  include/partition01.hpp
  include/polyline_road.hpp
  include/road_rep.hpp
  include/rtree.hpp
  include/rtree-impl.hpp
  include/sumo_network.hpp
  include/svg_helper.hpp
  include/xml_util.hpp
)

# Specify source Files
file(
  GLOB
  source_files
  src/arc_road.cpp  
  src/hwm_draw.cpp
  src/hwm_intersection.cpp
  src/hwm_lane.cpp
  src/hwm_network.cpp
  src/hwm_network_aux.cpp
  src/hwm_network_spatial.cpp
  src/hwm_road.cpp
  src/hwm_texture_gen.cpp
  src/hwm_xml_read.cpp
  src/hwm_xml_write.cpp
  src/libroad_common.cpp
  src/osm_draw.cpp
  src/osm_network.cpp
  src/osm_xml_read.cpp
  src/polyline_road.cpp 
  src/svg_helper.cpp
)

# Set output properties
set(STATIC FALSE)
if(STATIC)
add_library(Roadie STATIC ${source_files})
set_target_properties(Roadie PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Roadie/lib)
else(STATIC)
add_library(Roadie SHARED ${source_files})
set_target_properties(Roadie PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Roadie/lib)
endif(STATIC)

# Main includes
include_directories(./include/)

# Set external library links
set(EXTERNAL_LIBS
  ${GLIBMM2_LIBRARY}
  ${GLIB_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${LibXML++_LIBRARIES}
)

# Turn on proj.4 option in the code
if(TRUE)
add_definitions(-DHAVE_PROJ4)
find_package(PROJ4 REQUIRED)
if(NOT PROJ4_FOUND)
message(FATAL_ERROR "Package PROJ4 not found")
endif(NOT PROJ4_FOUND)
include_directories(${PROJ4_INCLUDE_DIR})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${PROJ4_LIBRARIES})
endif(TRUE)

# Link target to external libraries
target_link_libraries(Roadie ${EXTERNAL_LIBS})

# Set install targets
install(FILES ${ROADIE_HEADERS} DESTINATION /usr/local/include/Roadie/)
install(TARGETS Roadie DESTINATION /usr/local/lib)

# Turn on the debugging information
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -ggdb -fno-inline")

# Turn on the C++11 options
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_STD_CXX11)
    check_cxx_compiler_flag(--std=c++0x SUPPORTS_STD_CXX01)
    if(SUPPORTS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c++11")
    elseif(SUPPORTS_STD_CXX01)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++0x")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c++0x")
    else()
        message(ERROR "Compiler does not support --std=c++11 or --std=c++0x.")
    endif()
endif()
