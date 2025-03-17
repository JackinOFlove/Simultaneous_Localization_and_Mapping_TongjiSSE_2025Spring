#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

// 定义轨迹点结构
struct TrajectoryPoint {
    Vector3d position;
    Quaterniond orientation;
    double timestamp;
};

// 生成圆形轨迹
vector<TrajectoryPoint> generateCircularTrajectory(double radius, double height, int numPoints) {
    vector<TrajectoryPoint> trajectory;
    
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * M_PI * i / numPoints;
        double x = radius * cos(angle);
        double y = radius * sin(angle);
        double z = height;
        
        // 计算朝向（使机器人始终朝向圆心）
        double yaw = angle + M_PI; // 朝向圆心
        
        // 使用欧拉角创建四元数 (ZYX顺序)
        AngleAxisd rollAngle(0, Vector3d::UnitX());
        AngleAxisd pitchAngle(0, Vector3d::UnitY());
        AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
        Quaterniond q = yawAngle * pitchAngle * rollAngle;
        q.normalize();
        
        TrajectoryPoint point;
        point.position = Vector3d(x, y, z);
        point.orientation = q;
        point.timestamp = i * 0.1; // 假设每个点间隔0.1秒
        
        trajectory.push_back(point);
    }
    
    return trajectory;
}

// 将轨迹从一个坐标系转换到另一个坐标系
vector<TrajectoryPoint> transformTrajectory(const vector<TrajectoryPoint>& trajectory, 
                                           const Isometry3d& transformation) {
    vector<TrajectoryPoint> transformedTrajectory;
    
    for (const auto& point : trajectory) {
        TrajectoryPoint transformedPoint;
        
        // 转换位置
        transformedPoint.position = transformation * point.position;
        
        // 转换朝向
        transformedPoint.orientation = transformation.rotation() * point.orientation;
        
        // 时间戳保持不变
        transformedPoint.timestamp = point.timestamp;
        
        transformedTrajectory.push_back(transformedPoint);
    }
    
    return transformedTrajectory;
}

// 将轨迹保存到文件
void saveTrajectoryToFile(const vector<TrajectoryPoint>& trajectory, const string& filename) {
    ofstream file(filename);
    
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return;
    }
    
    file << "# timestamp x y z qw qx qy qz" << endl;
    
    for (const auto& point : trajectory) {
        file << point.timestamp << " "
             << point.position.x() << " "
             << point.position.y() << " "
             << point.position.z() << " "
             << point.orientation.w() << " "
             << point.orientation.x() << " "
             << point.orientation.y() << " "
             << point.orientation.z() << endl;
    }
    
    file.close();
    cout << "轨迹已保存到: " << filename << endl;
}

// 在控制台上可视化轨迹（简单的ASCII艺术）
void visualizeTrajectoryInConsole(const vector<TrajectoryPoint>& trajectory) {
    const int gridSize = 40;
    char grid[gridSize][gridSize];
    
    // 初始化网格
    for (int i = 0; i < gridSize; ++i) {
        for (int j = 0; j < gridSize; ++j) {
            grid[i][j] = ' ';
        }
    }
    
    // 找到轨迹的边界
    double minX = trajectory[0].position.x();
    double maxX = trajectory[0].position.x();
    double minY = trajectory[0].position.y();
    double maxY = trajectory[0].position.y();
    
    for (const auto& point : trajectory) {
        minX = min(minX, point.position.x());
        maxX = max(maxX, point.position.x());
        minY = min(minY, point.position.y());
        maxY = max(maxY, point.position.y());
    }
    
    // 添加一些边距
    double margin = 0.1 * max(maxX - minX, maxY - minY);
    minX -= margin;
    maxX += margin;
    minY -= margin;
    maxY += margin;
    
    // 将轨迹点映射到网格
    for (const auto& point : trajectory) {
        int i = static_cast<int>((point.position.x() - minX) / (maxX - minX) * (gridSize - 1));
        int j = static_cast<int>((point.position.y() - minY) / (maxY - minY) * (gridSize - 1));
        
        // 确保索引在有效范围内
        i = max(0, min(gridSize - 1, i));
        j = max(0, min(gridSize - 1, j));
        
        grid[i][j] = '*';
    }
    
    // 打印网格
    cout << "轨迹可视化 (俯视图, x-y平面):" << endl;
    cout << "x范围: [" << minX << ", " << maxX << "], y范围: [" << minY << ", " << maxY << "]" << endl;
    
    for (int j = gridSize - 1; j >= 0; --j) {
        for (int i = 0; i < gridSize; ++i) {
            cout << grid[i][j];
        }
        cout << endl;
    }
}

int main() {
    // 定义两个坐标系之间的变换，类似于coordinate_transform.cpp
    // 世界坐标系到机器人坐标系的变换
    Quaterniond q_W_R(0.8, 0.1, 0.2, 0.5); // w, x, y, z
    q_W_R.normalize();
    Vector3d t_W_R(1.0, 2.0, 0.5);
    
    Isometry3d T_W_R = Isometry3d::Identity();
    T_W_R.rotate(q_W_R.toRotationMatrix());
    T_W_R.pretranslate(t_W_R);
    
    // 生成机器人坐标系中的圆形轨迹
    double radius = 2.0;
    double height = 0.0;
    int numPoints = 100;
    
    vector<TrajectoryPoint> trajectory_R = generateCircularTrajectory(radius, height, numPoints);
    
    // 将轨迹从机器人坐标系转换到世界坐标系
    vector<TrajectoryPoint> trajectory_W = transformTrajectory(trajectory_R, T_W_R);
    
    // 保存轨迹到文件
    saveTrajectoryToFile(trajectory_R, "trajectory_robot.txt");
    saveTrajectoryToFile(trajectory_W, "trajectory_world.txt");
    
    // 在控制台上可视化轨迹
    cout << "机器人坐标系中的轨迹:" << endl;
    visualizeTrajectoryInConsole(trajectory_R);
    
    cout << "\n世界坐标系中的轨迹:" << endl;
    visualizeTrajectoryInConsole(trajectory_W);
    
    // 输出一些轨迹点的信息
    cout << "\n轨迹点示例 (机器人坐标系):" << endl;
    for (int i = 0; i < min(5, static_cast<int>(trajectory_R.size())); ++i) {
        cout << "点 " << i << ": 位置 = " << trajectory_R[i].position.transpose()
             << ", 朝向(四元数) = " << trajectory_R[i].orientation.coeffs().transpose() << endl;
    }
    
    cout << "\n轨迹点示例 (世界坐标系):" << endl;
    for (int i = 0; i < min(5, static_cast<int>(trajectory_W.size())); ++i) {
        cout << "点 " << i << ": 位置 = " << trajectory_W[i].position.transpose()
             << ", 朝向(四元数) = " << trajectory_W[i].orientation.coeffs().transpose() << endl;
    }
    
    return 0;
} 