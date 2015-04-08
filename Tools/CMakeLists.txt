cmake_minimum_required(VERSION 2.8)
project(Tools)

# Turn on cairo option in the code
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

find_package(LibXML++ REQUIRED)
if(NOT LibXML++_FOUND)
message(FATAL_ERROR "Package LibXML++ not found")
endif(NOT LibXML++_FOUND)
include_directories(${LibXML++_INCLUDE_DIRS})

find_package(FLTK REQUIRED)
if(NOT FLTK_FOUND)
message(FATAL_ERROR "Package FLTK not found")
endif(NOT FLTK_FOUND)
include_directories(${FLTK_INCLUDE_DIR})

# Adds project main code as an include directory.
include_directories(../Roadie/include/)

# And links
set(EXTERNAL_LIBS
  ${CMAKE_SOURCE_DIR}/Roadie/lib/libRoadie.so
  ${GLIBMM2_LIBRARY}
  ${GLIB_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${LibXML++_LIBRARIES}
  ${FLTK_LIBRARIES}
  #/usr/lib/x86_64-linux-gnu/libfltk.so
  #/usr/lib/x86_64-linux-gnu/libfltk_gl.so
)

# Source for making executables.
add_executable(view-network view-network.cpp)
add_executable(export-obj export-obj.cpp)
add_executable(import-osm import-osm.cpp)
add_executable(micro micro.cpp)

target_link_libraries(view-network ${EXTERNAL_LIBS})
target_link_libraries(export-obj ${EXTERNAL_LIBS})
target_link_libraries(import-osm ${EXTERNAL_LIBS})
target_link_libraries(micro ${EXTERNAL_LIBS})

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
