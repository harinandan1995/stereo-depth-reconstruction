cmake_minimum_required(VERSION 3.13)
project(stereo_depth_reconstruction)

# -> required packages
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})
# <- required packages

# -> set language and compiler optimization specific flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W -Wcast-qual -Wwrite-strings -Wextra") # -Werror
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=haswell -march=haswell -m64 -msse -msse2 -msse3 -msse4.1 -msse4.2 -mavx2 -mavx")
# <- set language and compiler optimization specific flags

# -> build config
if (NOT(EXISTS ${PROJECT_SOURCE_DIR}/bin AND IS_DIRECTORY ${PROJECT_SOURCE_DIR}/bin))
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
endif (NOT(EXISTS ${PROJECT_SOURCE_DIR}/bin AND IS_DIRECTORY ${PROJECT_SOURCE_DIR}/bin))
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

if (NOT(EXISTS ${PROJECT_SOURCE_DIR}/lib AND IS_DIRECTORY ${PROJECT_SOURCE_DIR}/lib))
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
endif (NOT(EXISTS ${PROJECT_SOURCE_DIR}/lib AND IS_DIRECTORY ${PROJECT_SOURCE_DIR}/lib))
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(BUILD_SHARED_LIBS ON)
# <- build config

# -> include headers
include_directories(${PROJECT_SOURCE_DIR}/include)

# -> build libs
add_library(block_matching STATIC src/BlockMatching.cpp src/DepthEstimation.cpp include/DepthEstimation.h src/Triangulation.cpp include/Triangulation.h)
add_library(stereo_dataset STATIC src/Dataset.cpp)
# <- build libs

add_executable(stereo_depth_reconstruction main.cpp)

# -> link
target_link_libraries(stereo_depth_reconstruction block_matching stereo_dataset opencv_core opencv_imgcodecs opencv_highgui)
# <- link
