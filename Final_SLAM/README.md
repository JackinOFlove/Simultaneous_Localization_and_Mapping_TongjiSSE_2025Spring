## 项目结构

项目分为三个主要部分：

### 1. Required_Section（传统特征匹配方法）

实现了三种经典特征提取与匹配算法（ORB、SIFT、SURF），以及基于这些特征的PnP（Perspective-n-Point）解算方法：

- `ORB.cpp`/`SIFT.cpp`/`SURF.cpp`：实现特征提取、匹配、三角化和深度误差计算
- `PnP_ORB.cpp`/`PnP_SIFT.cpp`/`PnP_SURF.cpp`：实现PnP算法估计相机位姿

### 2. DELTAS（深度学习方法）

基于ECCV 2020的DELTAS研究项目，实现了深度估计网络：

- `test_learnabledepth.py`：主要测试程序，包含网络推理部分
- `SUN3D_Dataprocess.py`：数据集转换程序，将SUN3D格式数据集转换为DELTAS可用的ScanNet格式
- `models/`：包含各种网络模型定义
  - `superpoint.py`：特征点检测与描述子生成
  - `triangulation.py`：特征点三角化
  - `densedepth.py`：稀疏到稠密的深度图生成
- `assets/`：包含数据处理模块和测试数据

### 3. mit_w85k1（部分数据集）

包含RGB-D数据集，用于测试以上算法：

- `image/`：RGB图像
- `depth/`：深度图
- `intrinsics.txt`：相机内参矩阵
- `extrinsics/`：相机外参

## 环境要求

### C++部分
- CMake >= 3.10
- OpenCV >= 4.0
- C++17支持的编译器

### Python部分
- Python 3 >= 3.5
- PyTorch >= 1.3.1
- Torchvision >= 0.4.2
- OpenCV >= 3.4 
- NumPy >= 1.18
- Path >= 15.0.0

## 编译与运行

### 编译C++程序

```bash
cd Required_Section
mkdir build && cd build
cmake ..
make
```

### 运行C++程序

编译完成后在build目录下执行：

```bash
# 运行ORB特征匹配
./ORB

# 运行SIFT特征匹配
./SIFT

# 运行SURF特征匹配
./SURF

# 运行PnP相机姿态估计
./PnP_ORB
./PnP_SIFT
./PnP_SURF
```

### 运行Python DELTAS模型

首先安装依赖：

```bash
pip install numpy opencv-python path torch torchvision
```

下载预训练模型并放置在DELTAS/assets目录下（来源：https://drive.google.com/uc?export=download&id=1lWjjl44o81m1_lZ2e9CW99OjQhZdd9gN）

运行测试：

```bash
cd DELTAS
python test_learnabledepth.py
```

## 算法原理

### 传统特征匹配方法

1. 特征提取与匹配：使用ORB/SIFT/SURF算法提取关键点并计算描述子
2. 姿态估计：基于特征匹配计算本质矩阵并分解得到相机运动（R, t）
3. 三角化：基于特征匹配和相机姿态计算3D点云
4. 评估：将三角化得到的深度与传感器获取的深度图进行比较，计算误差

### DELTAS深度学习方法

1. 使用SuperPoint网络提取稀疏特征点并计算描述子
2. 采用可微分三角化模块计算稀疏深度值
3. 利用稀疏到稠密深度估计网络生成完整的深度图
