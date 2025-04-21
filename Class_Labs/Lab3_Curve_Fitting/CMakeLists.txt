cmake_minimum_required(VERSION 3.10)
project(Lab3_Curve_Fitting)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Eigen3包
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # 如果标准方式找不到，尝试手动设置路径
  set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
endif()

# 查找Ceres包
find_package(Ceres REQUIRED)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 包含OpenCV, Ceres和Eigen3头文件目录
include_directories(${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# 添加可执行文件
add_executable(CurveFitting CurveFitting.cpp)

# 链接OpenCV和Ceres库
target_link_libraries(CurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES}) 