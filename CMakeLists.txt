project(pointcloud_registration)
cmake_minimum_required(VERSION 3.5)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)
find_package(Sophus REQUIRED)
# set(CMAKE_PREFIX_PATH "/home/gj/opencv/opencv-3.4.10/install/")
find_package(OpenCV REQUIRED)
# find_package(fmt REQUIRED)
# find_package(TBB REQUIRED COMPONENTS tbb)
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

option(BUILD_WITH_UBUNTU1804 OFF)

if(BUILD_WITH_UBUNTU1804)
    function(extract_file filename extract_dir)
        message(STATUS "Extract ${filename} to ${extract_dir} ...")
        set(temp_dir ${extract_dir})
        if(EXISTS ${temp_dir})
            file(REMOVE_RECURSE ${temp_dir})
        endif()
        file(MAKE_DIRECTORY ${temp_dir})
        execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xvzf ${filename}
                WORKING_DIRECTORY ${temp_dir})
    endfunction()

    set(TBB_ROOT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
    set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
    set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

    extract_file(${PROJECT_SOURCE_DIR}/thirdparty/tbb/2019_U8.tar.gz ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8)

    include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)

    #message(STATUS "======TBB_BUILD_DIR = ${TBB_BUILD_DIR}")
    #message(STATUS "======TBB_BUILD_PREFIX = ${TBB_BUILD_PREFIX}")

    tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
            compiler=gcc-9
            stdver=c++17
            ${TBB_BUILD_DIR}
            ${TBB_BUILD_PREFIX}
            CONFIG_DIR
            TBB_DIR)

    find_package(TBB REQUIRED)

    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
    link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)

    set(third_party_libs
            # ${catkin_LIBRARIES}
            # ${g2o_libs}
            # ${OpenCV_LIBS}
            # ${PCL_LIBRARIES}
            # ${Pangolin_LIBRARIES}
            # glog gflags
            # ${yaml-cpp_LIBRARIES}
            # yaml-cpp
            TBB::tbb
            )
else()
    set(third_party_libs
            # ${catkin_LIBRARIES}
            # ${g2o_libs}
            # ${OpenCV_LIBS}
            # ${PCL_LIBRARIES}
            # ${Pangolin_LIBRARIES}
            # glog gflags
            # ${yaml-cpp_LIBRARIES}
            # yaml-cpp
            tbb
            )
endif ()


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
target_compile_options(main PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(main ${PCL_LIBRARIES} ${GLOG_LIBRARIES} Sophus::Sophus ${OpenCV_LIBS}  ${OpenMP_CXX_LIBRARIES} ${third_party_libs})

