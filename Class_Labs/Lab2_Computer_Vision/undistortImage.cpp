#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <iostream>

using namespace std;

// 定义图像路径
vector<string> image_files = {
    //"../images/image1.jpg", 
    "../images/image2.jpg", 
    //"../images/image3.jpg"
};

// 手动实现去畸变
void undistortManual(const cv::Mat &image, const string &imageName) {
  cout << "正在手动实现去畸变: " << imageName << endl;
  
  // 畸变参数 - 进一步增强畸变系数的强度以便更明显地观察去畸变效果
  double k1 = -0.41040811, k2 = 0.12095907, p1 = 0.00028059, p2 = 2.10187114e-05;
  // 内参
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, image.type());

  // 计算去畸变后图像的内容
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      // 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted)
      double x = (u - cx) / fx, y = (v - cy) / fy;
      double r = sqrt(x * x + y * y);
      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

      // 赋值 (最近邻插值)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
        // 根据图像类型处理不同的像素格式
        if (image.type() == CV_8UC1) {
          image_undistort.at<uchar>(v, u) = image.at<uchar>((int)v_distorted, (int)u_distorted);
        } else if (image.type() == CV_8UC3) {
          image_undistort.at<cv::Vec3b>(v, u) = image.at<cv::Vec3b>((int)v_distorted, (int)u_distorted);
        }
      } else {
        if (image.type() == CV_8UC1) {
          image_undistort.at<uchar>(v, u) = 0;
        } else if (image.type() == CV_8UC3) {
          image_undistort.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 0);
        }
      }
    }
  }

  // 显示去畸变前后的图像
  cv::imshow("distorted", image);
  cv::imshow("undistorted (manual)", image_undistort);
  cv::waitKey(0);
}

// 使用OpenCV函数进行去畸变
void undistortOpenCV(const cv::Mat &image, const string &imageName) {
  cout << "正在使用OpenCV函数去畸变: " << imageName << endl;
  
  // 畸变参数 - 进一步增强畸变系数的强度以便更明显地观察去畸变效果
  double k1 = -0.41040811, k2 = 0.12095907, p1 = 0.00028059, p2 = 2.10187114e-05;
  // 内参
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
  
  // 创建相机矩阵
  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
                        fx, 0, cx,
                        0, fy, cy,
                        0, 0, 1);
  
  // 创建畸变系数矩阵
  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, 0);
  
  // 使用OpenCV函数进行去畸变
  cv::Mat image_undistort;
  cv::undistort(image, image_undistort, cameraMatrix, distCoeffs);
  
  // 显示结果
  cv::imshow("distorted", image);
  cv::imshow("undistorted (OpenCV)", image_undistort);
  cv::waitKey(0);
}

int main(int argc, char **argv) {
  cout << "将处理三张图片的去畸变，按任意键继续" << endl;
  
  // 遍历处理每张图片
  for (const auto &image_file : image_files) {
    cv::Mat image = cv::imread(image_file);
    
    if (image.data == nullptr) {
      cerr << "无法读取图像: " << image_file << endl;
      continue;
    }
    
    // 手动实现去畸变
    undistortManual(image, image_file);
    
    // 使用OpenCV函数进行去畸变
    undistortOpenCV(image, image_file);
    
    cv::destroyAllWindows();
  }
  
  cout << "所有图像去畸变处理完成" << endl;
  return 0;
}
