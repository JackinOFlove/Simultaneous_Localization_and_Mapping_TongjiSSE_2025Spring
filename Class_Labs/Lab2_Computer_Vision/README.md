# 图像处理实验

本实验基于OpenCV实现了以下功能：
1. 图像基础操作（读取、显示、像素遍历、复制、赋值）
2. 图像去畸变处理（手动实现和使用OpenCV函数）
3. 点云生成

## 环境要求

- Linux系统
- OpenCV库
- CMake
- C++编译器（如GCC）

## 安装OpenCV

在Ubuntu/Debian系统上安装OpenCV：

```bash
sudo apt-get update
sudo apt-get install -y libopencv-dev python3-opencv
```

验证安装：
```bash
pkg-config --modversion opencv4
```

## 编译和运行

### 编译

在项目根目录下创建build文件夹并编译：

```bash
mkdir -p build
cd build
cmake ..
make
```

### 运行

编译完成后，可以运行以下程序：

1. 图像基础操作：
```bash
./imageBasics
```

2. 图像去畸变：
```bash
./undistortImage
```

3. 点云生成：
```bash
./pointCloud
```

### 注意事项

程序中使用的图像路径是"../images/"，相对于build目录。请确保：
1. images目录位于Lab2目录下
2. 程序是在build目录中运行的

如果出现"文件不存在"的错误，请检查图像路径和文件是否存在。

## 文件说明

- `imageBasics.cpp` - 实现图像读取、显示、像素遍历、复制、赋值等基本操作
- `undistortImage.cpp` - 实现图像去畸变，包括手动实现和OpenCV函数调用
- `pointCloud.cpp` - 将图像转换为点云数据
- `images/` - 包含实验所用的图像文件 