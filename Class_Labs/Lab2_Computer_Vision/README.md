# 实验二：图像处理实验

本实验基于OpenCV实现了以下功能：
1. 图像基础操作（读取、显示、像素遍历、复制、赋值）
2. 图像去畸变处理（手动实现和使用OpenCV函数）
3. 点云生成

## 实验内容详解

### 1. 图像基础操作
- 读取和显示图像
- 获取图像的基本信息（宽、高、通道数）
- 遍历图像像素并访问每个像素的数据
- 测量图像处理操作的执行时间
- 演示 cv::Mat 的拷贝机制（浅拷贝与深拷贝）

### 2. 图像去畸变
- 手动实现去畸变算法，使用径向畸变和切向畸变模型
- 使用 OpenCV 内置函数进行去畸变
- 对比两种方法的效果和性能差异

### 3. 点云生成
- 使用相机内参和外参将图像转换为三维点云
- 通过图像强度生成伪深度信息
- 生成包含位置和颜色信息的点云数据
- 将点云数据保存为文本文件（X Y Z R G B格式）

## 环境要求

- Linux系统（推荐Ubuntu 18.04或更高版本）
- OpenCV库（4.x版本）
- CMake（3.0或更高版本）
- C++编译器（如GCC 7.5.0或更高版本）

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
- `CMakeLists.txt` - CMake配置文件

## 程序输出说明

### imageBasics
- 程序将依次处理三张图片
- 对每张图片，首先显示原始图像
- 显示图像的基本信息（宽、高、通道数）
- 输出遍历图像的耗时
- 演示浅拷贝（直接赋值）对原图的影响
- 演示深拷贝（使用clone）保持原图不变

### undistortImage
- 对每张图片进行两种不同的去畸变处理
- 显示原始畸变图像
- 显示使用手动实现的去畸变结果
- 显示使用OpenCV函数的去畸变结果
- 可以对比两种方法的效果差异

### pointCloud
- 对每张图片进行点云生成
- 显示原始图像、去畸变后的图像和伪深度图
- 生成点云数据并保存到文本文件
- 文件名格式为：原图文件名_pointcloud.txt
- 每行数据格式：X Y Z R G B（3D坐标和RGB颜色）

## 实验结果展示

程序运行后，会在images目录下生成多个点云数据文件：
- image1_pointcloud.txt
- image2_pointcloud.txt
- image3_pointcloud.txt

这些点云文件可以使用各种点云可视化工具（如CloudCompare、PCL Visualizer等）进行查看和分析。

## 参考资料

1. OpenCV官方文档：https://docs.opencv.org/
2. 《Computer Vision: Algorithms and Applications》, Richard Szeliski
3. 《Learning OpenCV》, Gary Bradski & Adrian Kaehler

