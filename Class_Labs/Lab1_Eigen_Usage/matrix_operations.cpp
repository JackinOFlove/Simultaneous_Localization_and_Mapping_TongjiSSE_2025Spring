#include <iostream>
#include <Eigen/Dense>
#include <random>

using namespace std;
using namespace Eigen;

int main() {
    // 设置随机数生成器
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> dist(-10.0, 10.0);
    
    // 创建一个3x3的随机矩阵
    Matrix3d A;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            A(i, j) = dist(gen);
        }
    }
    
    cout << "原始随机矩阵 A:" << endl;
    cout << A << endl << endl;
    
    // 1. 计算矩阵的转置
    Matrix3d A_transpose = A.transpose();
    cout << "矩阵的转置 A^T:" << endl;
    cout << A_transpose << endl << endl;
    
    // 2. 计算矩阵的迹（对角线元素之和）
    double trace = A.trace();
    cout << "矩阵的迹 tr(A): " << trace << endl << endl;
    
    // 3. 计算矩阵的逆（如果可逆）
    double det = A.determinant();
    cout << "矩阵的行列式 det(A): " << det << endl;
    
    if (abs(det) > 1e-10) {  // 检查矩阵是否可逆
        Matrix3d A_inverse = A.inverse();
        cout << "矩阵的逆 A^(-1):" << endl;
        cout << A_inverse << endl << endl;
        
        // 验证 A * A^(-1) = I
        Matrix3d I = A * A_inverse;
        cout << "验证 A * A^(-1):" << endl;
        cout << I << endl << endl;
    } else {
        cout << "矩阵不可逆" << endl << endl;
    }
    
    // 4. 计算矩阵所有元素的和
    double sum = A.sum();
    cout << "矩阵所有元素的和: " << sum << endl << endl;
    
    // 5. 计算矩阵的数乘（矩阵乘以一个标量）
    double scalar = 2.5;
    Matrix3d A_scaled = scalar * A;
    cout << "矩阵的数乘 " << scalar << " * A:" << endl;
    cout << A_scaled << endl << endl;
    
    // 额外：计算矩阵的特征值和特征向量
    EigenSolver<Matrix3d> es(A);
    cout << "矩阵的特征值:" << endl;
    cout << es.eigenvalues() << endl << endl;
    
    cout << "矩阵的特征向量:" << endl;
    cout << es.eigenvectors() << endl;
    
    return 0;
} 