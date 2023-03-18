cmake_minimum_required (VERSION 3.1)

project (object_seg)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

# put FindGLFW.cmake in the source directory of CMakeLists.txt
find_package(OpenCV 3.3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  {PROJECT_SOURCE_DIR}
  include
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/eigen3
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/MapObject.cc
src/ObjectDetector.cc
src/Parameters.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${PCL_LIBRARIES} 
)

add_executable(object_seg_node src/main_object_seg_node.cpp )
target_link_libraries(object_seg_node 
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)

add_executable(show_hypermap_node src/main_show_hypermap_node.cpp )
target_link_libraries(show_hypermap_node 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
)

# add_executable(test_node src/test_node.cpp )
# target_link_libraries(test_node 
#   ${PROJECT_NAME}
#   ${OpenCV_LIBS}
# )
