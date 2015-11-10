project(simple_object_detection)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
)

aux_source_directory(src/ SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries (${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
