CMAKE_MINIMUM_REQUIRED(VERSION 3.19)
PROJECT(CV_lab1)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Debug")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# PCL
FIND_PACKAGE(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES("/usr/include/eigen3")


SET(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

MESSAGE("[THRID_PARTY_LIBS] contains: ${THIRD_PARTY_LIBS}")

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/app)
