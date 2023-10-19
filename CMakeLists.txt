project(pointcloud_registration)
cmake_minimum_required(VERSION 2.8)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


find_package(OpenMP)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)
find_package(Sophus REQUIRED)
# set(CMAKE_PREFIX_PATH "/home/gj/opencv/opencv-3.4.10/install/")
find_package(OpenCV REQUIRED)
# find_package(fmt REQUIRED)
# find_package(TBB REQUIRED)
# include_directories(${TBB_INCLUDE_DIRS})
# link_directories(${TBB_LIBRARY_DIRS})

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


link_directories(
    ${PCL_LIBRARY_DIRS}
    
)

file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.cc")
# set(third_party_libs
#     tbb
#     fmt::fmt
# )
# add_library(${PROJECT_NAME}.ch7
#     src/icp_3d.cc
#     src/kdtree.cc
#     src/ndt_3d.cc
#     src/point_cloud_utils.cc)


# target_link_libraries(${PROJECT_NAME}.ch7 ${third_party_libs})

add_executable(main main.cpp  ${SOURCES})
target_link_libraries(main ${PCL_LIBRARIES} ${GLOG_LIBRARIES} Sophus::Sophus ${OpenCV_LIBS} tbb)

