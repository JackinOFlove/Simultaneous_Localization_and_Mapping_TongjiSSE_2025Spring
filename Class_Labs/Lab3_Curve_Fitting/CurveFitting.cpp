//
// Created by xiang on 18-11-19.
//

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <cmath>

using namespace std;

// 代价函数的计算模型
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

  // 残差的计算
  template<typename T>
  bool operator()(
    const T *const abc, // 模型参数，有3维
    T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
    return true;
  }

  const double _x, _y;    // x,y数据
};

// 将数字转换为科学计数法表示
std::string formatPowerOfTen(double value) {
    if (value == 0) return "0";
    
    int exponent = static_cast<int>(floor(log10(value)));
    
    // 对于10的整数次幂
    if (pow(10, exponent) == value) {
        return "10^" + std::to_string(exponent);
    }
    
    // 对于其他值，使用科学计数法
    std::stringstream ss;
    ss << std::scientific << std::setprecision(1) << value;
    return ss.str();
}

// 绘制网格线
void drawGrid(cv::Mat& image, int margin, int plot_width, int plot_height, 
              double x_min, double x_max, double y_min, double y_max,
              int num_grid = 10) {
    cv::Scalar grid_color(230, 230, 230);  // 浅灰色
    
    // 绘制竖直网格线
    for (int i = 1; i < num_grid; i++) {
        int x = margin + (i * plot_width) / num_grid;
        cv::line(image, cv::Point(x, margin), 
                cv::Point(x, image.rows - margin),
                grid_color, 1, cv::LINE_AA);
    }
    
    // 绘制水平网格线
    for (int i = 1; i < num_grid; i++) {
        int y = margin + (i * plot_height) / num_grid;
        cv::line(image, cv::Point(margin, y),
                cv::Point(image.cols - margin, y),
                grid_color, 1, cv::LINE_AA);
    }
}

// 绘制坐标轴刻度
void drawAxisValues(cv::Mat& image, int margin, int plot_width, int plot_height,
                   double x_min, double x_max, double y_min, double y_max,
                   int num_ticks = 5) {
    for (int i = 0; i <= num_ticks; i++) {
        // X轴刻度
        double x_val = x_min + (x_max - x_min) * i / num_ticks;
        int x_pos = margin + (i * plot_width) / num_ticks;
        std::stringstream ss;
        ss << fixed << setprecision(2) << x_val;  // 只在显示时控制精度
        cv::putText(image, ss.str(), 
                   cv::Point(x_pos - 20, image.rows - margin + 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        
        // Y轴刻度 - 使用科学计数法
        double y_val = y_min + (y_max - y_min) * (num_ticks - i) / num_ticks;
        int y_pos = margin + (i * plot_height) / num_ticks;
        string y_label = formatPowerOfTen(y_val);
        cv::putText(image, y_label,
                   cv::Point(margin - 65, y_pos + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
}

// 绘制图例
void drawLegend(cv::Mat& image, int margin) {
    cv::Rect legend_box(image.cols - margin - 150, margin + 10, 140, 50);
    cv::rectangle(image, legend_box, cv::Scalar(245, 245, 245), -1);  // 填充浅灰色背景
    cv::rectangle(image, legend_box, cv::Scalar(0, 0, 0), 1);        // 黑色边框
    
    // 散点图例
    cv::circle(image, cv::Point(legend_box.x + 15, legend_box.y + 15),
               3, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    cv::putText(image, "Data Points",
                cv::Point(legend_box.x + 30, legend_box.y + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    
    // 拟合曲线图例
    cv::line(image, cv::Point(legend_box.x + 10, legend_box.y + 35),
             cv::Point(legend_box.x + 20, legend_box.y + 35),
             cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::putText(image, "Fitted Curve",
                cv::Point(legend_box.x + 30, legend_box.y + 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
}

// 绘制拟合结果
void plotResults(const vector<double>& x_data, const vector<double>& y_data, const double abc[3]) {
    // 创建图像
    cv::Mat image = cv::Mat::zeros(800, 1000, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));  // 设置白色背景

    // 设置X轴范围为0-2.00
    double x_min = 0.0;
    double x_max = 2.0;
    
    // 设置Y轴为对数刻度，使用10的幂次
    double y_ticks[] = {1.0, 1.0e2, 1.0e4, 1.0e6, 1.0e8, 1.0e10};
    double y_min = y_ticks[0];
    double y_max = y_ticks[5];

    // 设置边距
    int margin = 80;  // 增加边距以容纳坐标轴标签
    int plot_width = image.cols - 2 * margin;
    int plot_height = image.rows - 2 * margin;

    // 绘制网格
    drawGrid(image, margin, plot_width, plot_height, x_min, x_max, y_min, y_max);

    // 绘制坐标轴
    cv::line(image, cv::Point(margin, image.rows - margin),
             cv::Point(image.cols - margin, image.rows - margin),
             cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::line(image, cv::Point(margin, image.rows - margin),
             cv::Point(margin, margin),
             cv::Scalar(0, 0, 0), 2, cv::LINE_AA);

    // 绘制Y轴刻度 - 使用指定的对数刻度
    for (int i = 0; i < 6; i++) {
        double y_val = y_ticks[i];
        int y_pos = image.rows - margin - (log10(y_val) - log10(y_min)) * plot_height / (log10(y_max) - log10(y_min));
        
        // 绘制水平网格线
        cv::line(image, cv::Point(margin, y_pos),
                cv::Point(image.cols - margin, y_pos),
                cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
        
        // 绘制刻度值
        string y_label = "10^" + std::to_string(static_cast<int>(log10(y_val)));
        cv::putText(image, y_label,
                   cv::Point(margin - 65, y_pos + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
    
    // 绘制X轴刻度
    for (int i = 0; i <= 5; i++) {
        double x_val = x_min + (x_max - x_min) * i / 5;
        int x_pos = margin + (i * plot_width) / 5;
        std::stringstream ss;
        ss << fixed << setprecision(2) << x_val;
        cv::putText(image, ss.str(), 
                   cv::Point(x_pos - 20, image.rows - margin + 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    // 绘制散点 - 使用对数坐标，减小点的大小
    for (size_t i = 0; i < x_data.size(); i++) {
        if (x_data[i] >= x_min && x_data[i] <= x_max && y_data[i] > 0) {
            int x = margin + (x_data[i] - x_min) * plot_width / (x_max - x_min);
            int y = image.rows - margin - (log10(y_data[i]) - log10(y_min)) * plot_height / (log10(y_max) - log10(y_min));
            if (y >= margin && y <= image.rows - margin) {
                cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
            }
        }
    }

    // 绘制拟合曲线 - 使用对数坐标
    vector<cv::Point> curve_points;
    const int num_points = plot_width;  // 使用更多的点使曲线更平滑
    for (int i = 0; i < num_points; i++) {
        double x = x_min + (x_max - x_min) * i / (num_points - 1);
        double y = exp(abc[0] * x * x + abc[1] * x + abc[2]);  // 使用完整精度计算
        
        // 确保y值在可绘制范围内
        if (y > 0 && y >= y_min && y <= y_max) {
            int plot_x = margin + i;
            int plot_y = image.rows - margin - (log10(y) - log10(y_min)) * plot_height / (log10(y_max) - log10(y_min));
            if (plot_y >= margin && plot_y <= image.rows - margin) {
                curve_points.push_back(cv::Point(plot_x, plot_y));
            }
        }
    }
    
    // 使用折线段绘制曲线
    for (size_t i = 1; i < curve_points.size(); i++) {
        cv::line(image, curve_points[i-1], curve_points[i],
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    // 添加标题和标签
    cv::putText(image, "Curve Fitting Result",
                cv::Point(margin + 300, margin - 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    
    // 添加坐标轴标签
    cv::putText(image, "X-axis",
                cv::Point(image.cols - margin + 10, image.rows - margin + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::putText(image, "Y-axis",
                cv::Point(margin - 70, margin - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

    // 添加图例
    drawLegend(image, margin);
    
    // 添加拟合曲线的函数式
    std::stringstream func_ss;
    func_ss << "y = exp(" << fixed << setprecision(5) << abc[0] << "x^2 + " 
            << abc[1] << "x + " << abc[2] << ")";
    cv::putText(image, func_ss.str(),
                cv::Point(margin + 50, margin + 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

    // 保存图像
    cv::imwrite("function_fit.png", image);
}

int main(int argc, char **argv) {
    // 初始估计参数值
    double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
    double abc[3] = {ae, be, ce};

    // 读取数据
    vector<double> x_data, y_data;      // 数据
    ifstream file("Data.csv");
    if (!file.is_open()) {
        cerr << "Cannot open file Data.csv" << endl;
        return 1;
    }

    string line;
    int index = 0;
    while (getline(file, line)) {
        if (!line.empty()) {
            try {
                double y = stod(line);  // 保持原始精度
                // 生成从0.01到2.00的x值，共200个点 (0.01, 0.02, 0.03, ..., 2.00)
                double x = 0.01 + index * 0.01;  // 保持原始精度
                x_data.push_back(x);
                y_data.push_back(y);
                index++;
            } catch (const std::invalid_argument& e) {
                cerr << "Cannot convert data: " << line << endl;
                continue;
            }
        }
    }
    file.close();

    if (x_data.empty()) {
        cerr << "No valid data read" << endl;
        return 1;
    }

    cout << "Successfully read " << x_data.size() << " data points" << endl;
    cout << "X range: " << x_data.front() << " to " << x_data.back() << endl;

    // 构建最小二乘问题
    ceres::Problem problem;
    for (size_t i = 0; i < x_data.size(); i++) {
        problem.AddResidualBlock(     // 向问题中添加误差项
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,            // 核函数，这里不使用，为空
            abc                 // 待估计参数
        );
    }

    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    
    // 输出求解时间
    cout << "solve time cost = " << fixed << setprecision(5) << time_used.count() << " seconds. " << endl;
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c = ";
    for (auto a:abc) cout << fixed << setprecision(5) << a << " ";
    cout << endl;

    // 输出拟合函数
    cout << "\nFitted function: y = exp(" 
         << fixed << setprecision(5) << abc[0] << "x^2 + " 
         << abc[1] << "x + " 
         << abc[2] << ")" << endl;

    // 输出每个数据点的拟合值和实际值
    cout << "\nComparison of fitted and actual values:" << endl;
    
    // 计算MSE, NRMSE和R平方
    double sum_squared_error = 0.0;
    double sum_squared_total = 0.0;
    double mean_y = 0.0;
    
    // 首先计算y的平均值
    for (size_t i = 0; i < y_data.size(); i++) {
        mean_y += y_data[i];
    }
    mean_y /= y_data.size();
    
    // 计算误差和总方差
    for (size_t i = 0; i < x_data.size(); i++) {
        double fitted_y = exp(abc[0] * x_data[i] * x_data[i] + abc[1] * x_data[i] + abc[2]);
        double error = y_data[i] - fitted_y;
        sum_squared_error += error * error;
        sum_squared_total += (y_data[i] - mean_y) * (y_data[i] - mean_y);
        
        cout << "x=" << fixed << setprecision(2) << x_data[i] 
             << ", fitted=" << setprecision(5) << fitted_y 
             << ", actual=" << y_data[i] << endl;
    }
    
    // 计算MSE (Mean Squared Error)
    double mse = sum_squared_error / x_data.size();
    
    // 计算NRMSE (Normalized Root Mean Squared Error)
    double nrmse = sqrt(mse) / (y_data.back() - y_data.front());
    
    // 计算R平方 (R-squared)
    double r_squared = 1.0 - (sum_squared_error / sum_squared_total);
    
    // 输出评估指标
    cout << "\nEvaluation Metrics:" << endl;
    cout << "MSE (Mean Squared Error): " << scientific << setprecision(5) << mse << endl;
    cout << "NRMSE (Normalized Root Mean Squared Error): " << fixed << setprecision(5) << nrmse << endl;
    cout << "R-squared: " << fixed << setprecision(5) << r_squared << endl;
    // 绘制结果
    plotResults(x_data, y_data, abc);
    cout << "\nPlot saved as 'function_fit.png'" << endl;

    return 0;
}