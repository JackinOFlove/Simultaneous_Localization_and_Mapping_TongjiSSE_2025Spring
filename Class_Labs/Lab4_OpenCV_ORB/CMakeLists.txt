cmake_minimum_required(VERSION 3.10)
project(Lab4_OpenCV_ORB)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 包含OpenCV头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 创建两个可执行文件
add_executable(orb_cv orb_cv.cpp)
add_executable(orb_self orb_self.cpp)

# 链接OpenCV库
target_link_libraries(orb_cv ${OpenCV_LIBS})
target_link_libraries(orb_self ${OpenCV_LIBS})

# 输出编译信息
message(STATUS "OpenCV库版本: ${OpenCV_VERSION}")
message(STATUS "包含目录: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "链接库: ${OpenCV_LIBS}")

# 创建输出目录
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output_cv)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output_self)

# 复制图像文件夹到构建目录
file(COPY ${CMAKE_SOURCE_DIR}/images DESTINATION ${CMAKE_BINARY_DIR})

# 设置安装规则
install(TARGETS orb_cv orb_self
        RUNTIME DESTINATION bin) 