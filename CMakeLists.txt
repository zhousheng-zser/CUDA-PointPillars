# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.7)
project(pointpillar)
set(CMAKE_BUILD_PYTHON $ENV{USE_Python})
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

find_package(CUDA REQUIRED)

set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wextra -Wall -Wno-missing-field-initializers -Wno-deprecated-declarations -O3")
set(CMAKE_CXX_FLAGS_DEBUG   "-std=c++17 -O0 -g")

set(CUDA_NVCC_FLAGS_RELEASE "-Werror=all-warnings -Xcompiler -std=c++17,-Wextra,-Wall,-Wno-deprecated-declarations,-O3")
set(CUDA_NVCC_FLAGS_DEBUG   "-std=c++17 -O0 -g")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} 
    -gencode arch=compute_$ENV{CUDASM},code=compute_$ENV{CUDASM}
)

# Linker flags to handle missing NVDLA compiler library (may be optional on Jetson)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

include_directories(
    $ENV{CUDA_Inc}
    $ENV{TensorRT_Inc}
    src
	src/pointpillar
)

link_directories(
    $ENV{CUDA_Lib}
    $ENV{TensorRT_Lib}
    /usr/lib/aarch64-linux-gnu/nvidia
    build
)

file(GLOB_RECURSE CORE_FILES 
    src/pointpillar/*.cu 
    src/pointpillar/*.cpp
    src/common/tensor.cu
    src/common/tensorrt.cpp
)

####################### libpointpillar_core.so ##########################
cuda_add_library(pointpillar_core SHARED 
  	${CORE_FILES}
)


target_link_libraries(pointpillar_core
	libnvinfer.so
	nvdla_runtime
)

# Set RPATH for runtime library search
set_target_properties(pointpillar_core PROPERTIES
	INSTALL_RPATH "/usr/lib/aarch64-linux-gnu/nvidia"
	BUILD_WITH_INSTALL_RPATH TRUE
)

# Create a version of pointpillar_core with old ABI for receiver-inno
cuda_add_library(pointpillar_core_inno SHARED 
  	${CORE_FILES}
)

# Use old ABI for pointpillar_core_inno
target_compile_definitions(pointpillar_core_inno PRIVATE _GLIBCXX_USE_CXX11_ABI=0)

target_link_libraries(pointpillar_core_inno
	libnvinfer.so
	nvdla_runtime
)

# Set RPATH for runtime library search
set_target_properties(pointpillar_core_inno PROPERTIES
	INSTALL_RPATH "/usr/lib/aarch64-linux-gnu/nvidia"
	BUILD_WITH_INSTALL_RPATH TRUE
)
#######################################################################


####################### pointpillar program #############################
cuda_add_executable(pointpillar 
	src/main.cpp
)

target_link_libraries(pointpillar
	pointpillar_core
	nvdla_runtime
)

# Set RPATH for runtime library search
set_target_properties(pointpillar PROPERTIES
	INSTALL_RPATH "/usr/lib/aarch64-linux-gnu/nvidia"
	BUILD_WITH_INSTALL_RPATH TRUE
)
#######################################################################

file(GLOB_RECURSE DETECT_FILES
    src/detect/*.hpp 
    src/detect/*.cpp
)
####################### receiver-hsai program #############################


cuda_add_executable(receiver-hsai 
	src/receiver-hsai.cpp
    ${DETECT_FILES}
)
target_include_directories(receiver-hsai 
    PRIVATE   
    /home/zser/HesaiLidar_SDK_2.0/sdk
    src/include
)
target_link_directories(receiver-hsai
    PRIVATE
    /home/zser/HesaiLidar_SDK_2.0/build
)

target_link_libraries(receiver-hsai
	pointpillar_core
	nvdla_runtime
    hesai_sdk_wrapper
)

# Set RPATH for runtime library search
set_target_properties(receiver-hsai PROPERTIES
	INSTALL_RPATH "/usr/lib/aarch64-linux-gnu/nvidia"
	BUILD_WITH_INSTALL_RPATH TRUE
)
#######################################################################


####################### receiver-inno program #############################

cuda_add_executable(receiver-inno 
	src/receiver-inno.cpp
    ${DETECT_FILES}
)
# Disable C++11 ABI to match inno_sdk_wrapper.so (compiled with old ABI)
target_compile_definitions(receiver-inno PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
target_include_directories(receiver-inno 
    PRIVATE   
    /home/zser/inno-lidar/apps/example
    /home/zser/inno-lidar
    src/include
)
target_link_directories(receiver-inno
    PRIVATE
    /home/zser/inno-lidar/build_unix/apps/example
    /home/zser/inno-lidar/lib
)

target_link_libraries(receiver-inno
	pointpillar_core_inno
	nvdla_runtime
    inno_sdk_wrapper
    libinnolidarsdkclient.a
    libinnolidarsdkcommon.a
    libinnolidarutils.a
    pthread
)

# Set RPATH for runtime library search
set_target_properties(receiver-inno PROPERTIES
	INSTALL_RPATH "/usr/lib/aarch64-linux-gnu/nvidia"
	BUILD_WITH_INSTALL_RPATH TRUE
)
#######################################################################
