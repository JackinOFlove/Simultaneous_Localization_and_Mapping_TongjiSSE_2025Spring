# 实验一：Eigen库使用教程

本项目展示了Eigen库在机器人和SLAM领域的基本应用，包括矩阵运算、坐标变换、轨迹生成和可视化等功能。

## 1. Eigen库简介

Eigen是一个高性能的C++模板库，用于线性代数、矩阵和向量运算、几何变换等数学操作。它被广泛应用于机器人、计算机视觉、SLAM等领域。Eigen具有以下特点：

- 高效的矩阵和向量运算
- 丰富的线性代数功能
- 几何变换（旋转、平移等）
- 四元数和欧拉角处理
- 纯头文件实现，无需链接外部库

## 2. 安装和配置Eigen库

### Ubuntu/Debian系统

```bash
# 安装Eigen库
sudo apt-get update
sudo apt-get install libeigen3-dev

# 验证安装
pkg-config --modversion eigen3
```

### 其他系统

- **macOS**：`brew install eigen`
- **Windows**：可以从[官方网站](https://eigen.tuxfamily.org/)下载源码，或使用vcpkg：`vcpkg install eigen3`

## 3. 项目结构

本项目包含以下主要文件：

- `matrix_operations.cpp`：演示Eigen库的基本矩阵操作
- `coordinate_transform.cpp`：演示坐标变换计算
- `trajectory_visualization.cpp`：演示轨迹生成和可视化
- `orthogonal_matrix_check.cpp`：检查旋转矩阵是否为正交矩阵
- `CMakeLists.txt`：CMake构建配置文件

## 4. 编译和构建项目

### 创建构建目录

```bash
cd /root/Slam_Class_Labs/Lab1_Eigen_Usage
mkdir -p build
cd build
```

### 使用CMake生成构建系统

```bash
cmake ..
```

### 编译项目

```bash
make
```

如果一切顺利，将生成以下可执行文件：
- `matrix_operations`
- `coordinate_transform`
- `trajectory_visualization`
- `orthogonal_matrix_check`

## 5. 运行程序

### 矩阵基本操作

```bash
./matrix_operations
```

**功能**：演示Eigen库的基本矩阵操作，包括：
- 创建随机矩阵
- 计算矩阵的转置
- 计算矩阵的迹（对角线元素之和）
- 计算矩阵的逆
- 计算矩阵所有元素的和
- 计算矩阵的数乘（矩阵乘以标量）
- 计算矩阵的特征值和特征向量

### 坐标变换

```bash
./coordinate_transform
```

**功能**：演示坐标变换计算，包括：
- 定义两个坐标系（小萝卜一号和小萝卜二号）
- 使用四元数表示旋转
- 构建变换矩阵
- 将点从一个坐标系转换到另一个坐标系

### 轨迹可视化

```bash
./trajectory_visualization
```

**功能**：演示轨迹生成和可视化，包括：
- 生成圆形轨迹
- 坐标系变换
- 将轨迹保存到文件
- 在控制台中可视化轨迹（ASCII艺术）

### 正交矩阵检查

```bash
./orthogonal_matrix_check
```

**功能**：检查旋转矩阵是否为正交矩阵，包括：
- 创建基于欧拉角的旋转矩阵
- 计算矩阵的转置、迹、逆等属性
- 验证矩阵是否满足正交矩阵的性质（R^T * R = I）
- 检查矩阵的行列式是否为±1

## 6. 常见问题解决

### 找不到Eigen库

如果CMake无法找到Eigen库，可以尝试以下方法：

1. 确保已正确安装Eigen库
2. 设置Eigen库的路径：
   ```bash
   export Eigen3_DIR=/path/to/eigen3
   ```
3. 或在CMakeLists.txt中手动指定Eigen库路径：
   ```cmake
   set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
   include_directories(${EIGEN3_INCLUDE_DIR})
   ```

### 编译错误

如果遇到编译错误，请检查：
1. C++编译器是否支持C++11或更高标准
2. Eigen库是否正确安装
3. 代码中是否有语法错误

## 7. 进一步学习

- [Eigen官方文档](https://eigen.tuxfamily.org/dox/)
- [Eigen快速参考](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)
- [几何模块教程](https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html)

## 8. 许可证

本项目使用MIT许可证。Eigen库使用MPL2许可证。 