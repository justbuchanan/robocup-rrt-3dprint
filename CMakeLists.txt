cmake_minimum_required(VERSION 3.2.0)
project("RoboCup RRT 3dprint")

include(ExternalProject)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# setup ccache to speed up recompiles.  It's especially useful when switching back and forth
# between branches where you'd normally have to recompile things many times.
# see http://stackoverflow.com/questions/1815688/how-to-use-ccache-with-cmake
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

# use rrt library from github
set(rrt_install_dir ${CMAKE_CURRENT_BINARY_DIR}/rrt)
set(rrt_lib_path ${rrt_install_dir}/lib/libRRT.a)
ExternalProject_Add(rrt_project
    GIT_REPOSITORY "https://github.com/robojackets/rrt"
    GIT_TAG "more-birrt-tests"
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${rrt_install_dir}
    BUILD_BYPRODUCTS ${rrt_lib_path}
)
set_target_properties(rrt_project PROPERTIES EXCLUDE_FROM_ALL TRUE)

# add rrt library target.  IMPORTED means that it is created externally
add_library(rrt STATIC IMPORTED)
set_property(TARGET rrt PROPERTY IMPORTED_LOCATION ${rrt_lib_path})
add_dependencies(rrt rrt_project)

# include rrt library headers
include_directories(${rrt_install_dir}/include/)

# main executable
add_executable(rrtgen main.cpp)
target_link_libraries(rrtgen rrt)
