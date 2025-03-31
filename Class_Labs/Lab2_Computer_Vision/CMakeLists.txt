cmake_minimum_required(VERSION 3.0)
project(Lab2)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# 查找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(imageBasics imageBasics.cpp)
add_executable(undistortImage undistortImage.cpp)
add_executable(pointCloud pointCloud.cpp)

# 链接OpenCV库
target_link_libraries(imageBasics ${OpenCV_LIBS})
target_link_libraries(undistortImage ${OpenCV_LIBS})
target_link_libraries(pointCloud ${OpenCV_LIBS}) 