cmake_minimum_required(VERSION 3.10)
project(FeatureMatchingDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(ORB ORB.cpp)
target_link_libraries(ORB ${OpenCV_LIBS})

add_executable(SIFT SIFT.cpp)
target_link_libraries(SIFT ${OpenCV_LIBS})

add_executable(SURF SURF.cpp)
target_link_libraries(SURF ${OpenCV_LIBS}) 

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(SURF ${OpenCV_LIBS})
