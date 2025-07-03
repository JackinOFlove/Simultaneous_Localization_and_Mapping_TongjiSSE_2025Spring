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

add_executable(PnP_SIFT PnP_SIFT.cpp)
target_link_libraries(Pnp_SIFT ${OpenCV_LIBS})

add_executable(PnP_ORB PnP_ORB.cpp)
target_link_libraries(Pnp_ORB ${OpenCV_LIBS})

add_executable(PnP_SURF PnP_SURF.cpp)
target_link_libraries(Pnp_SURF ${OpenCV_LIBS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(SURF ${OpenCV_LIBS})
