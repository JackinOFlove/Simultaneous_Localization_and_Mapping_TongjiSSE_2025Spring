#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

// 定义图像路径
vector<string> image_files = {
    "../images/image1.jpg", 
    "../images/image2.jpg", 
    "../images/image3.jpg"
};

// 相机内参
const double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

// 畸变参数
const double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;

/**
 * 将二维像素坐标转换为三维点云
 * 
 * 这里实现一个简单的伪深度图生成和点云构建
 * 在实际应用中，通常需要立体视觉或其他深度测量方法来获取深度信息
 */
void generatePointCloud(const cv::Mat &image, const string &imageName) {
    cout << "正在为图像生成点云: " << imageName << endl;

    // 首先进行图像去畸变
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
                          fx, 0, cx,
                          0, fy, cy,
                          0, 0, 1);
    
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, 0);
    
    cv::Mat undistorted;
    cv::undistort(image, undistorted, cameraMatrix, distCoeffs);
    
    // 转换为灰度图，用于获取伪深度信息
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(undistorted, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = undistorted.clone();
    }
    
    // 创建一个简单的伪深度图（这里用像素强度作为深度值的演示）
    // 在实际应用中，深度应该来自深度传感器或立体视觉
    cv::Mat depth;
    cv::GaussianBlur(gray, depth, cv::Size(5, 5), 0);
    
    // 定义输出文件
    string filename = imageName.substr(0, imageName.find_last_of('.')) + "_pointcloud.txt";
    ofstream outfile(filename);
    
    if (!outfile.is_open()) {
        cerr << "无法创建点云文件: " << filename << endl;
        return;
    }
    
    // 生成点云数据
    // 为了简化，我们只对部分像素生成点云（间隔采样）
    int step = 10; // 采样步长
    
    for (int v = 0; v < undistorted.rows; v += step) {
        for (int u = 0; u < undistorted.cols; u += step) {
            // 计算归一化像素坐标
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            
            // 获取伪深度值（将像素值映射到合理的深度范围，例如0.5到5米）
            double z = 0.5 + 4.5 * (depth.at<uchar>(v, u) / 255.0);
            
            // 构建3D点
            double X = x * z;
            double Y = y * z;
            double Z = z;
            
            // 获取颜色信息
            int b = 0, g = 0, r = 0;
            if (undistorted.channels() == 3) {
                cv::Vec3b color = undistorted.at<cv::Vec3b>(v, u);
                b = color[0];
                g = color[1];
                r = color[2];
            } else {
                b = g = r = undistorted.at<uchar>(v, u);
            }
            
            // 保存点云数据（X Y Z R G B格式）
            outfile << X << " " << Y << " " << Z << " " 
                   << r << " " << g << " " << b << endl;
        }
    }
    
    outfile.close();
    cout << "点云已保存到：" << filename << endl;
    
    // 显示原始图像和处理后的图像
    cv::imshow("Original", image);
    cv::imshow("Undistorted", undistorted);
    cv::imshow("Depth", depth);
    cv::waitKey(0);
}

int main(int argc, char **argv) {
    cout << "将处理三张图片并生成点云，按任意键继续" << endl;
    
    // 处理每张图片
    for (const auto &image_file : image_files) {
        cv::Mat image = cv::imread(image_file);
        
        if (image.data == nullptr) {
            cerr << "无法读取图像: " << image_file << endl;
            continue;
        }
        
        // 生成点云
        generatePointCloud(image, image_file);
        
        cv::destroyAllWindows();
    }
    
    cout << "所有点云生成完成" << endl;
    return 0;
} 