# 实验四: ORB特征提取与匹配

## 实验概述

本实验实现了ORB(Oriented FAST and Rotated BRIEF)特征点的提取和匹配算法。实验包含两个核心部分：OpenCV的ORB实现和自定义的ORB实现。通过对比这两种实现，可以深入理解ORB算法的原理和实现细节。

## 实验目的

1. 学习ORB特征提取算法的原理与实现
2. 掌握特征点检测、描述和匹配的完整流程
3. 理解OpenCV提供的特征处理接口和用法
4. 通过手动实现ORB算法，加深对计算机视觉算法的理解

## 环境配置

### 依赖项

- C++17或更高版本
- OpenCV 4.x
- CMake 3.10或更高版本

### 编译方法

```bash
mkdir -p build
cd build
cmake ..
make
```

## 代码结构

项目包含以下主要文件：

- `orb_cv.cpp`: 使用OpenCV提供的ORB接口实现特征提取和匹配
- `orb_self.cpp`: 自定义实现的ORB特征提取和匹配算法
- `CMakeLists.txt`: CMake构建配置文件
- `images/`: 包含测试图像的文件夹（1.png至6.png）

## 使用方法

编译后，在build目录下会生成两个可执行文件：

1. **OpenCV实现版本**:
   ```bash
   ./orb_cv
   ```
   输出文件保存在`output_cv`目录中

2. **自定义实现版本**:
   ```bash
   ./orb_self
   ```
   输出文件保存在`output_self`目录中

当前配置下，程序会自动处理以下图像对：
- 1.png和2.png
- 3.png和4.png
- 5.png和6.png

### 输出结果

OpenCV实现版本会生成以下输出：
- `keypoints_<图像名>.png`: 每张图像的特征点可视化
- `matches_<图像1>_<图像2>.png`: 原始匹配结果
- `goodmatches_<图像1>_<图像2>.png`: 筛选后的匹配结果

自定义实现版本会生成：
- `match_<图像1>_<图像2>.png`: 匹配结果

## 算法原理

### ORB算法简介

ORB(Oriented FAST and Rotated BRIEF)算法是一种高效的特征提取算法，由以下两部分组成：

1. **特征点检测**: 使用FAST(Features from Accelerated Segment Test)算法检测角点，然后计算Harris角点响应值进行排序

2. **特征描述**: 使用旋转感知的BRIEF(Binary Robust Independent Elementary Features)描述子，可以提高对旋转的鲁棒性

### OpenCV实现

OpenCV实现使用了库提供的接口：
- `cv::ORB::create()`: 创建ORB检测器和描述子提取器
- `detector->detect()`: 检测关键点
- `descriptor->compute()`: 计算描述子
- `matcher->match()`: 进行特征匹配
- 通过距离阈值筛选匹配点对

### 自定义实现

自定义实现包含以下步骤：
- 使用OpenCV的FAST检测角点
- 自定义计算关键点的主方向
- 使用预定义的ORB模式计算二进制描述子
- 使用汉明距离(Hamming Distance)进行特征匹配
- 通过距离阈值筛选匹配结果

## 参数设置

- **特征点数量**: 由FAST算法的阈值控制，较低的阈值会检测到更多特征点
- **匹配阈值**: 
  - OpenCV版本: 使用`max(2.5 * min_dist, 35.0)`作为筛选阈值
  - 自定义版本: 使用固定阈值`d_max = 60`

## 实验结果分析

通过比较这两种实现的结果，可以发现：

1. **特征点分布**:
   - OpenCV实现倾向于在图像的高对比度区域检测更多特征点
   - 自定义实现的特征点分布可能更加均匀，但数量通常较少

2. **匹配质量**:
   - OpenCV实现通常能提供更精确的匹配
   - 自定义实现的匹配结果可能包含更多误匹配，但在良好纹理区域仍能提供不错的结果

3. **运行效率**:
   - OpenCV实现经过优化，运行速度通常更快
   - 自定义实现更易于理解和修改，但效率较低

## 可能的改进方向

1. **特征点筛选**: 可以结合Harris角点响应值进行更精细的特征点筛选
2. **描述子改进**: 可以尝试其他二进制描述子，如BRISK或FREAK
3. **匹配策略**: 可以实现比率测试(Ratio Test)来进一步提高匹配质量
4. **几何验证**: 添加RANSAC算法进行几何一致性检查，去除外点
5. **并行计算**: 使用OpenMP或GPU加速大量特征点的计算

## 参考资料

1. Rublee, E., Rabaud, V., Konolige, K., & Bradski, G. (2011). ORB: An efficient alternative to SIFT or SURF. In 2011 International conference on computer vision (pp. 2564-2571). IEEE.
2. OpenCV文档: https://docs.opencv.org/4.x/db/d95/classcv_1_1ORB.html
3. FAST角点检测: https://docs.opencv.org/4.x/df/d0c/tutorial_py_fast.html 