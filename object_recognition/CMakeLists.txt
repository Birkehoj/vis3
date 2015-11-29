cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(vis3)
#set( CMAKE_BUILD_TYPE Debug)
set( CMAKE_BUILD_TYPE Release)
find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )
include_directories(
  ${PCL_INCLUDE_DIRS}
  src
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable (object_recognition
  src/object_recognition_types.h
  src/feature_cloud.h
  src/object_recognition.h
  src/object.h
  src/object.cpp
  src/feature_cloud.cpp
  src/object_recognition.cpp  
  src/main.cpp
)

target_link_libraries (object_recognition
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Compiler flags
# with this: The program has unexpectedly finished.
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
#if(HAS_FLAGS)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
#endif()
