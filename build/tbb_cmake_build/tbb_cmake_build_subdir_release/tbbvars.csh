#!/bin/csh
setenv TBBROOT "/home/gj/c++_study/test/pointcloud_registration/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8" #
setenv tbb_bin "/home/gj/c++_study/test/pointcloud_registration/build/tbb_cmake_build/tbb_cmake_build_subdir_release" #
if (! $?CPATH) then #
    setenv CPATH "${TBBROOT}/include" #
else #
    setenv CPATH "${TBBROOT}/include:$CPATH" #
endif #
if (! $?LIBRARY_PATH) then #
    setenv LIBRARY_PATH "${tbb_bin}" #
else #
    setenv LIBRARY_PATH "${tbb_bin}:$LIBRARY_PATH" #
endif #
if (! $?LD_LIBRARY_PATH) then #
    setenv LD_LIBRARY_PATH "${tbb_bin}" #
else #
    setenv LD_LIBRARY_PATH "${tbb_bin}:$LD_LIBRARY_PATH" #
endif #
 #
