CMAKE_MINIMUM_REQUIRED(VERSION 3.19)
PROJECT(stereo_calib)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_STANDARD 14)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(fmt REQUIRED)

ADD_SUBDIRECTORY(./app)