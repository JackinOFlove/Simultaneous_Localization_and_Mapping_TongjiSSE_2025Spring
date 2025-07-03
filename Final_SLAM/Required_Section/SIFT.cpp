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

void find_SIFT_feature_matches(
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

void pose_estimation_2d2d(
    const std::vector<KeyPoint> &keypoints_1,
    const std::vector<KeyPoint> &keypoints_2,
    const std::vector<DMatch> &matches,
    Mat &R, Mat &t)
{
  // 提供的内参矩阵
  Mat K = (Mat_<double>(3, 3) << 570.3422047415297129191458225250244140625, 0, 320,
           0, 570.3422047415297129191458225250244140625, 240,
           0, 0, 1);

  //-- 把匹配点转换为vector<Point2f>的形式
  vector<Point2f> points1;
  vector<Point2f> points2;

  for (int i = 0; i < (int)matches.size(); i++)
  {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }

  //-- 计算本质矩阵
  Mat essential_matrix;
  essential_matrix = findEssentialMat(points1, points2, K);

  //-- 从本质矩阵中恢复旋转和平移信息.
  recoverPose(essential_matrix, points1, points2, K, R, t);
}

Point2f pixel2cam(const Point2d &p, const Mat &K)
{
  return Point2f(
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void triangulation(
    const vector<KeyPoint> &keypoint_1,
    const vector<KeyPoint> &keypoint_2,
    const std::vector<DMatch> &matches,
    const Mat &R, const Mat &t,
    vector<Point3d> &points,
    vector<int> &query_indices) // 添加参数来保存第一帧特征点索引
{
  Mat T1 = (Mat_<float>(3, 4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

  // 提供的内参矩阵
  Mat K = (Mat_<double>(3, 3) << 570.3422047415297129191458225250244140625, 0, 320,
           0, 570.3422047415297129191458225250244140625, 240,
           0, 0, 1);
  vector<Point2f> pts_1, pts_2;
  query_indices.clear(); // 清空索引向量

  for (DMatch m : matches)
  {
    // 将像素坐标转换至相机坐标
    pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
    pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    query_indices.push_back(m.queryIdx); // 记录第一帧中的特征点索引
  }

  Mat pts_4d;
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // 转换成非齐次坐标
  points.clear(); // 确保点集是空的
  for (int i = 0; i < pts_4d.cols; i++)
  {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); // 归一化
    Point3d p(
        x.at<float>(0, 0),
        x.at<float>(1, 0),
        x.at<float>(2, 0));
    points.push_back(p);
  }
}

// 修改函数参数，添加匹配关系和关键点参数
Mat calculateDepthError(
    const vector<Point3d> &points3D,
    const Mat &depthMap,
    const Mat &K,
    const vector<KeyPoint> &keypoints_1,
    const vector<int> &query_indices)
{
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  Mat depthErrors(1, points3D.size(), CV_64F);

  // 定义最大允许误差阈值（单位：米）
  const double MAX_ERROR_THRESHOLD = 2;

  for (int i = 0; i < points3D.size(); ++i)
  {
    // 获取第一帧中的特征点像素坐标
    KeyPoint kp = keypoints_1[query_indices[i]];
    int u = cvRound(kp.pt.x);
    int v = cvRound(kp.pt.y);

    // 三角化得到的深度
    double Z = points3D[i].z;

    if (u >= 0 && u < depthMap.cols && v >= 0 && v < depthMap.rows)
    {
      // 从深度图获取真实深度（以米为单位）
      double depth = static_cast<double>(depthMap.at<unsigned short>(v, u) >> 3) / 1000.0;
      if (depth > 0)
      {
        double error = abs(Z - depth);
        // 过滤掉误差太大的点
        if (error <= MAX_ERROR_THRESHOLD)
        {
          depthErrors.at<double>(i) = error;
        }
        else
        {
          depthErrors.at<double>(i) = NAN;
        }
      }
      else
      {
        depthErrors.at<double>(i) = NAN;
      }
    }
    else
    {
      depthErrors.at<double>(i) = NAN;
    }
  }
  return depthErrors;
}

void computeErrors(const Mat &depthErrors, const string &featureType, double &mae, double &rmse, double &rmse_log)
{
  mae = 0;
  rmse = 0;
  rmse_log = 0;
  int count = 0;

  for (int i = 0; i < depthErrors.cols; ++i)
  {
    double error = depthErrors.at<double>(i);
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
    find_SIFT_feature_matches(img1, img2, keypoints1, keypoints2, matches);

    if (matches.size() < 8)
    {
      cerr << "Not enough matching points" << endl;
      continue;
    }

    // 估计相机位姿
    Mat R, t;
    pose_estimation_2d2d(keypoints1, keypoints2, matches, R, t);

    // 三角化
    vector<Point3d> points;
    vector<int> query_indices; // 保存第一帧关键点索引
    triangulation(keypoints1, keypoints2, matches, R, t, points, query_indices);

    // 计算深度误差（与第一帧深度图比较）
    Mat depthErrors = calculateDepthError(points, depth1, K, keypoints1, query_indices);

    // 计算误差指标
    double mae, rmse, rmse_log;
    computeErrors(depthErrors, "SIFT", mae, rmse, rmse_log);

    // 累计有效的误差值
    if (!isnan(mae) && !isnan(rmse) && !isnan(rmse_log))
    {
      total_mae += mae;
      total_rmse += rmse;
      total_rmse_log += rmse_log;
      valid_pairs++;

      // 定义高质量图像对的标准（例如：MAE < 0.2米）
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
  int max_pairs = 200;

  processImagesAndDepths(imagePaths, depthPaths, K, interval, max_pairs);

  // 结束计时并计算总时间
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  cout << "\nTotal execution time: " << duration.count() / 1000.0 << " seconds" << endl;

  return 0;
}

void find_SIFT_feature_matches(const Mat &img_1, const Mat &img_2,
                               std::vector<KeyPoint> &keypoints_1,
                               std::vector<KeyPoint> &keypoints_2,
                               std::vector<DMatch> &matches)
{
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  Ptr<SIFT> sift = SIFT::create();

  //-- 第一步:检测 SIFT 关键点位置
  sift->detect(img_1, keypoints_1);
  sift->detect(img_2, keypoints_2);

  //-- 第二步:根据关键点位置计算 SIFT 描述子
  sift->compute(img_1, keypoints_1, descriptors_1);
  sift->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的SIFT描述子进行匹配，使用 L2 距离
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
  vector<DMatch> match;
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  // 找出所有匹配之间的最小距离和最大距离，即最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    double dist = match[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  // 当描述子之间的距离大于两倍的最小距离时，即认为匹配有误。但有时候最小距离会非常小，设置一个经验值30作为下限
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (match[i].distance <= max(2 * min_dist, 30.0))
    {
      matches.push_back(match[i]);
    }
  }
}