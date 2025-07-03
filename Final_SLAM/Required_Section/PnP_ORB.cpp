#include <iostream>
#include <vector>
#include <string>
#include <filesystem> // C++17 标准库
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <chrono> // 添加chrono库用于计时

namespace fs = std::filesystem;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

void find_ORB_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

void readFilenamesFromFolder(const std::string &folderPath, std::vector<std::string> &filenames)
{
    for (const auto &entry : fs::directory_iterator(folderPath))
    {
        if (entry.is_regular_file())
        {
            filenames.push_back(entry.path().string());
        }
    }
    // 按文件名排序
    sort(filenames.begin(), filenames.end());
}

Mat readDepth(const std::string &path)
{
    Mat depth = imread(path, IMREAD_UNCHANGED);
    if (depth.empty())
    {
        cerr << "Error: Unable to load depth image at " << path << endl;
        exit(EXIT_FAILURE);
    }
    return depth;
}

// 将像素坐标转换为相机坐标
Point2f pixel2cam(const Point2d &p, const Mat &K)
{
    return Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

// 使用PnP方法进行位姿估计
void pose_estimation_3d2d(
    const vector<Point3d> &points_3d,
    const vector<Point2f> &points_2d,
    const Mat &K,
    Mat &R, Mat &t)
{
    // 初始化输出的旋转向量和平移向量
    Mat r;

    // 使用RANSAC方法求解PnP问题
    solvePnPRansac(points_3d, points_2d, K, Mat(), r, t);

    // 将旋转向量转换为旋转矩阵
    Rodrigues(r, R);
}

// 从深度图获取3D点
vector<Point3d> get_3d_points_from_depth(
    const vector<KeyPoint> &keypoints,
    const Mat &depth_map,
    const Mat &K)
{
    vector<Point3d> points_3d;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for (const auto &kp : keypoints)
    {
        int u = cvRound(kp.pt.x);
        int v = cvRound(kp.pt.y);

        if (u >= 0 && u < depth_map.cols && v >= 0 && v < depth_map.rows)
        {
            unsigned short depth_value = depth_map.at<unsigned short>(v, u);

            // 将深度值转换为米
            double z = static_cast<double>(depth_value >> 3) / 1000.0;

            if (z > 0)
            {
                // 反投影: 从像素坐标和深度计算3D点
                double x = (u - cx) * z / fx;
                double y = (v - cy) * z / fy;
                points_3d.push_back(Point3d(x, y, z));
            }
            else
            {
                // 对于无效深度，添加一个NaN点
                points_3d.push_back(Point3d(NAN, NAN, NAN));
            }
        }
        else
        {
            points_3d.push_back(Point3d(NAN, NAN, NAN));
        }
    }

    return points_3d;
}

// 计算重投影误差
Mat calculateReprojectionError(
    const vector<Point3d> &points_3d,
    const vector<Point2f> &points_2d,
    const Mat &R, const Mat &t,
    const Mat &K)
{
    Mat errors(1, points_3d.size(), CV_64F);

    // 定义最大允许误差阈值（像素）
    const double MAX_ERROR_THRESHOLD = 10.0;

    vector<Point2f> reprojected_points;
    projectPoints(points_3d, R, t, K, Mat(), reprojected_points);

    for (int i = 0; i < points_3d.size(); ++i)
    {
        if (isnan(points_3d[i].x) || isnan(points_3d[i].y) || isnan(points_3d[i].z))
        {
            errors.at<double>(i) = NAN;
            continue;
        }

        double error = norm(points_2d[i] - reprojected_points[i]);

        if (error <= MAX_ERROR_THRESHOLD)
        {
            errors.at<double>(i) = error;
        }
        else
        {
            errors.at<double>(i) = NAN;
        }
    }

    return errors;
}

void computeErrors(const Mat &errors, const string &featureType, double &mae, double &rmse, double &rmse_log)
{
    mae = 0;
    rmse = 0;
    rmse_log = 0;
    int count = 0;

    for (int i = 0; i < errors.cols; ++i)
    {
        double error = errors.at<double>(i);
        if (!isnan(error))
        {
            mae += error;
            rmse += error * error;
            rmse_log += log(1 + error) * log(1 + error);
            ++count;
        }
    }
    if (!count)
    {
        cout << "Error: No valid points for calculating errors" << endl;
        mae = rmse = rmse_log = NAN;
        return;
    }

    mae /= count;
    rmse = sqrt(rmse / count);
    rmse_log = sqrt(rmse_log / count);

    cout << "=====" << featureType << " Feature Results=====" << endl;
    cout << "Abs: " << mae << endl;
    cout << "RMSE: " << rmse << endl;
    cout << "RMSE log: " << rmse_log << endl;
    cout << "Valid points: " << count << endl;
    cout << "====================" << endl;
}

void processImagesAndDepths(const vector<string> &imagePaths, const vector<string> &depthPaths, const Mat &K, int interval, int max_pairs)
{
    // 累计误差和计数变量
    double total_mae = 0;
    double total_rmse = 0;
    double total_rmse_log = 0;
    int valid_pairs = 0;
    int total_processed_pairs = 0;
    int high_quality_pairs = 0;

    // 高质量图像对的误差累计
    double hq_total_mae = 0;
    double hq_total_rmse = 0;
    double hq_total_rmse_log = 0;

    // 处理图像对
    for (size_t i = 0; i < imagePaths.size() - interval && valid_pairs < max_pairs; i += 1)
    {
        cout << "Processing image pair " << i << " and " << i + interval << endl;
        total_processed_pairs++;

        // 读取图像
        Mat img1 = imread(imagePaths[i]);
        Mat img2 = imread(imagePaths[i + interval]);
        if (img1.empty() || img2.empty())
        {
            cerr << "Failed to load images" << endl;
            continue;
        }

        // 读取深度图（使用第一帧的深度图作为ground truth）
        Mat depth1 = readDepth(depthPaths[i]);

        // 特征提取和匹配
        vector<KeyPoint> keypoints1, keypoints2;
        vector<DMatch> matches;
        find_ORB_feature_matches(img1, img2, keypoints1, keypoints2, matches);

        if (matches.size() < 8)
        {
            cerr << "Not enough matching points" << endl;
            continue;
        }

        // 准备PnP所需的数据
        vector<Point2f> points_2d;
        vector<int> query_indices;

        for (const DMatch &m : matches)
        {
            points_2d.push_back(keypoints2[m.trainIdx].pt);
            query_indices.push_back(m.queryIdx);
        }

        // 从第一帧的深度图获取3D点
        vector<Point3d> points_3d;
        for (int idx : query_indices)
        {
            KeyPoint kp = keypoints1[idx];
            int u = cvRound(kp.pt.x);
            int v = cvRound(kp.pt.y);

            if (u >= 0 && u < depth1.cols && v >= 0 && v < depth1.rows)
            {
                unsigned short depth_value = depth1.at<unsigned short>(v, u);
                double z = static_cast<double>(depth_value >> 3) / 1000.0;

                if (z > 0)
                {
                    // 反投影: 从像素坐标和深度计算3D点
                    double x = (u - K.at<double>(0, 2)) * z / K.at<double>(0, 0);
                    double y = (v - K.at<double>(1, 2)) * z / K.at<double>(1, 1);
                    points_3d.push_back(Point3d(x, y, z));
                }
                else
                {
                    points_3d.push_back(Point3d(NAN, NAN, NAN));
                }
            }
            else
            {
                points_3d.push_back(Point3d(NAN, NAN, NAN));
            }
        }

        // 过滤无效的3D点
        vector<Point3d> valid_points_3d;
        vector<Point2f> valid_points_2d;

        for (size_t j = 0; j < points_3d.size(); j++)
        {
            if (!isnan(points_3d[j].x) && !isnan(points_3d[j].y) && !isnan(points_3d[j].z))
            {
                valid_points_3d.push_back(points_3d[j]);
                valid_points_2d.push_back(points_2d[j]);
            }
        }

        if (valid_points_3d.size() < 4)
        {
            cerr << "Not enough valid 3D-2D correspondences for PnP" << endl;
            continue;
        }

        // 使用PnP估计相机位姿
        Mat R, t;
        pose_estimation_3d2d(valid_points_3d, valid_points_2d, K, R, t);

        // 计算重投影误差
        Mat reprojErrors = calculateReprojectionError(valid_points_3d, valid_points_2d, R, t, K);

        // 计算误差指标
        double mae, rmse, rmse_log;
        computeErrors(reprojErrors, "PnP-ORB", mae, rmse, rmse_log);

        // 累计有效的误差值
        if (!isnan(mae) && !isnan(rmse) && !isnan(rmse_log))
        {
            total_mae += mae;
            total_rmse += rmse;
            total_rmse_log += rmse_log;
            valid_pairs++;

            // 定义高质量图像对的标准（例如：MAE < 0.5像素）
            if (mae < 0.5)
            {
                high_quality_pairs++;
                // 累计高质量图像对的误差
                hq_total_mae += mae;
                hq_total_rmse += rmse;
                hq_total_rmse_log += rmse_log;
            }
        }
    }

    // 计算并输出平均误差
    if (valid_pairs > 0)
    {
        cout << "\n============Overall Average Results============" << endl;
        cout << "Average Abs: " << (total_mae / valid_pairs) << endl;
        cout << "Average RMSE: " << (total_rmse / valid_pairs) << endl;
        cout << "Average RMSE log: " << (total_rmse_log / valid_pairs) << endl;
        cout << "Valid image pairs: " << valid_pairs << "/" << max_pairs << endl;
        cout << "High quality image pairs: " << high_quality_pairs << endl;
        cout << "Total processed pairs: " << total_processed_pairs << endl;

        // 输出高质量图像对的平均误差
        if (high_quality_pairs > 0)
        {
            cout << "\n============High Quality Pairs Results============" << endl;
            cout << "HQ Average Abs: " << (hq_total_mae / high_quality_pairs) << endl;
            cout << "HQ Average RMSE: " << (hq_total_rmse / high_quality_pairs) << endl;
            cout << "HQ Average RMSE log: " << (hq_total_rmse_log / high_quality_pairs) << endl;
        }
        cout << "==========================================" << endl;
    }
    else
    {
        cout << "No valid image pairs for calculating average errors" << endl;
    }
}

int main()
{
    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();

    string imageFolderPath = "/root/Final_SLAM/mit_w85k1/whole_apartment/image";
    string depthFolderPath = "/root/Final_SLAM/mit_w85k1/whole_apartment/depth";

    vector<string> imagePaths;
    vector<string> depthPaths;

    readFilenamesFromFolder(imageFolderPath, imagePaths);
    readFilenamesFromFolder(depthFolderPath, depthPaths);

    cout << "Files loaded:" << endl;
    cout << "RGB images: " << imagePaths.size() << endl;
    cout << "Depth images: " << depthPaths.size() << endl;

    // 使用提供的内参矩阵
    Mat K = (Mat_<double>(3, 3) << 570.3422047415297129191458225250244140625, 0, 320,
             0, 570.3422047415297129191458225250244140625, 240,
             0, 0, 1);

    // 可以设置间隔和最大处理对数
    int interval = 1;
    int max_pairs = 100; // 使用与ORB.cpp相同的最大对数

    processImagesAndDepths(imagePaths, depthPaths, K, interval, max_pairs);

    // 结束计时并计算总时间
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    cout << "\nTotal execution time: " << duration.count() / 1000.0 << " seconds" << endl;

    return 0;
}

void find_ORB_feature_matches(const Mat &img_1, const Mat &img_2,
                              std::vector<KeyPoint> &keypoints_1,
                              std::vector<KeyPoint> &keypoints_2,
                              std::vector<DMatch> &matches)
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    // 找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}