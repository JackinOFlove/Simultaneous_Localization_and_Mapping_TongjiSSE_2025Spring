cmake_minimum_required(VERSION 3.0)
project(Lab1_Eigen_Usage)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Eigen库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 如果找不到Eigen3，可以尝试使用这种方式
if(NOT EIGEN3_FOUND)
  # 尝试使用pkg-config查找
  find_package(PkgConfig)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

# 添加可执行文件
add_executable(matrix_operations matrix_operations.cpp)
add_executable(coordinate_transform coordinate_transform.cpp)
add_executable(trajectory_visualization trajectory_visualization.cpp)
add_executable(orthogonal_matrix_check orthogonal_matrix_check.cpp)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 输出信息
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}") 