# 实验三：使用Ceres优化器进行曲线拟合

## 简介

本项目展示了使用Ceres求解器优化库进行高级曲线拟合的技术。它旨在通过最小化残差平方和，将函数拟合到带有噪声的数据点。该实现支持拟合形式如下的指数二次模型：

```
y = exp(a*x^2 + b*x + c)
```

这个模型对于呈现快速指数增长的数据特别有效。程序在拟合二次函数之前对数据进行对数变换，然后将结果转换回原始空间。

## 特点

- 从CSV文件导入数据
- 对数变换以更好地拟合指数数据
- 使用Ceres求解器进行高级优化
- 计算拟合质量指标（MSE、NRMSE、R²）
- 使用适当的坐标轴缩放可视化数据点和拟合曲线
- 可自定义的可视化选项

## 程序说明

1. **CurveFitting.cpp**
   - 改进版的曲线拟合实现
   - 提供更专业的可视化效果
   - 包含以下增强功能：
     * 精确的坐标轴刻度
     * 网格线显示
     * 图例说明
     * 数据点和拟合曲线的清晰标识

## 依赖项

本项目需要以下库：

- 支持C++14的C++编译器
- [Ceres求解器](http://ceres-solver.org/)（2.0或更高版本）
- [Eigen](http://eigen.tuxfamily.org/)（3.3或更高版本）
- [OpenCV](https://opencv.org/)（4.0或更高版本）
- CMake（3.10或更高版本）

## 安装

### 在Ubuntu上安装依赖项

```bash
# 安装CMake和构建工具
sudo apt-get update
sudo apt-get install -y cmake build-essential

# 安装Eigen库
sudo apt-get install -y libeigen3-dev

# 安装Ceres依赖项
sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev

# 安装Ceres
# 选项1：从包管理器安装
sudo apt-get install -y libceres-dev

# 选项2：从源代码安装（如果需要最新版本）
# git clone https://github.com/ceres-solver/ceres-solver.git
# cd ceres-solver
# mkdir build && cd build
# cmake ..
# make -j4
# sudo make install

# 安装OpenCV
sudo apt-get install -y libopencv-dev
```

### 构建项目

```bash
# 克隆仓库（或下载源代码）
# git clone <仓库URL>
# cd Lab3_Curve_Fitting

# 创建构建目录
mkdir -p build
cd build

# 配置和构建
cmake ..
make
```

## 使用方法

### 准备数据

将数据放在名为`Data.csv`的CSV文件中，并放置在项目根目录下。该文件应包含单列数值。x值将自动生成为从0开始的连续整数。

### 运行程序

从构建目录中，您可以选择运行以下任一程序：

运行基础版本：
```bash
./CurveFitting
```

### 输出

程序输出：
- 拟合参数（a、b、c）
- 对数空间和原始空间的函数方程
- 误差指标（MSE、NRMSE、R²）
- 数据和拟合曲线的图形表示

增强版本（ceresFitting）额外提供：
- 带有网格线的专业图表
- 清晰的图例说明
- 完整的坐标轴刻度

## 工作原理

1. **数据变换**：程序对y值应用对数变换，将指数增长模式转换为更适合用二次函数近似的形式。

2. **优化**：使用Ceres求解器，程序优化参数（a、b、c）以最小化对数变换数据与二次函数之间的平方差和。

3. **质量评估**：程序计算：
   - 均方误差（MSE）：实际值与预测值之间平方差的平均值
   - 归一化均方根误差（NRMSE）：均方根误差除以数据范围
   - 决定系数（R²）：衡量模型解释数据方差的程度

4. **可视化**：程序创建一个图表，同时显示原始数据点和拟合曲线，使用对数缩放以更好地显示指数数据。

## 结果解读

- **R²值**：范围从0到1，值越接近1表示拟合越好。一般而言，高于0.9的值被认为是优秀的。
- **NRMSE**：值越低表示拟合越好。通常低于0.1的值被认为是良好的。
- **视觉检查**：曲线应该通过或接近大多数数据点，捕捉整体趋势。

## 自定义

您可以修改代码以：
- 更改拟合函数
- 调整优化参数
- 修改可视化设置
- 处理不同的数据格式

## 许可证

本项目是开源的，可根据MIT许可证使用。

## 相关

- 谷歌的[Ceres求解器](http://ceres-solver.org/)
- 用于可视化的[OpenCV](https://opencv.org/)
