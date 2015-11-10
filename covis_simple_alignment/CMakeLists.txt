project(covis_simple_alignment)
cmake_minimum_required(VERSION 2.8)
set( CMAKE_BUILD_TYPE Debug)
find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(COVIS HINTS $ENV{HOME}/covis/build)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${COVIS_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
  ${COVIS_DEFINITIONS}
)

aux_source_directory(src/ SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries (${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${COVIS_LIBRARIES}
)

