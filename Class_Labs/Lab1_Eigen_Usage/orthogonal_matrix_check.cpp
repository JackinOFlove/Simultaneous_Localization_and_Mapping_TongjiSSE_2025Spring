#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;

int main() {
    // 将角度转换为弧度
    double alpha = 30.0 * M_PI / 180.0;  // 30度
    double beta = 50.0 * M_PI / 180.0;   // 50度
    double gamma = 40.0 * M_PI / 180.0;  // 40度
    
    // 创建三个基本旋转矩阵
    Matrix3d Rx, Ry, Rz;
    
    // 绕X轴旋转矩阵
    Rx << 1, 0, 0,
          0, cos(alpha), -sin(alpha),
          0, sin(alpha), cos(alpha);
    
    // 绕Y轴旋转矩阵
    Ry << cos(beta), 0, sin(beta),
          0, 1, 0,
          -sin(beta), 0, cos(beta);
    
    // 绕Z轴旋转矩阵
    Rz << cos(gamma), -sin(gamma), 0,
          sin(gamma), cos(gamma), 0,
          0, 0, 1;
    
    // 计算最终的旋转矩阵 R = Rx * Ry * Rz
    Matrix3d R = Rz * Ry * Rx;
    
    // 输出旋转矩阵
    cout << "旋转矩阵 R:" << endl;
    cout << R << endl << endl;
    
    // 输出矩阵的转置
    Matrix3d R_transpose = R.transpose();
    cout << "矩阵的转置 R^T:" << endl;
    cout << R_transpose << endl << endl;
    
    // 输出矩阵的迹
    double trace = R.trace();
    cout << "矩阵的迹 tr(R): " << trace << endl << endl;
    
    // 输出矩阵的逆
    Matrix3d R_inverse = R.inverse();
    cout << "矩阵的逆 R^(-1):" << endl;
    cout << R_inverse << endl << endl;
    
    // 计算所有元素和
    double sum = R.sum();
    cout << "矩阵所有元素和: " << sum << endl << endl;
    
    // 数乘（以2为例）
    double scalar = 2.0;
    Matrix3d R_scaled = scalar * R;
    cout << "矩阵的数乘 " << scalar << " * R:" << endl;
    cout << R_scaled << endl << endl;
    
    // 检查是否为正交矩阵
    // 对于正交矩阵，R^T * R 应该等于单位矩阵
    Matrix3d I = Matrix3d::Identity();
    Matrix3d RtR = R.transpose() * R;
    
    // 计算R^T * R与单位矩阵的差
    Matrix3d diff = RtR - I;
    double error = diff.norm();
    
    cout << "R^T * R:" << endl;
    cout << RtR << endl << endl;
    
    cout << "R^T * R - I:" << endl;
    cout << diff << endl << endl;
    
    cout << "误差范数: " << error << endl << endl;
    
    // 设置一个小的阈值来判断是否为正交矩阵
    double threshold = 1e-10;
    if (error < threshold) {
        cout << "R 是正交矩阵" << endl;
    } else {
        cout << "R 不是正交矩阵" << endl;
    }
    
    // 另一种检查方法：计算行列式
    double det = R.determinant();
    cout << "R 的行列式: " << det << endl;
    
    // 正交矩阵的行列式应该是 +1 或 -1
    if (abs(abs(det) - 1.0) < threshold) {
        cout << "R 的行列式接近 " << (det > 0 ? "+1" : "-1") << "，符合正交矩阵特性" << endl;
    } else {
        cout << "R 的行列式不接近 ±1，不符合正交矩阵特性" << endl;
    }
    
    return 0;
} 