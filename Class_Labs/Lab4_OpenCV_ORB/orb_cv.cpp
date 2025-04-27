#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <filesystem>

using namespace std;
using namespace cv;

void process_image_pair(const string &img1_path, const string &img2_path, const string &output_path) {
  cout << "Processing: " << img1_path << " and " << img2_path << endl;
  
  //-- 读取图像
  Mat img_1 = imread(img1_path, cv::IMREAD_COLOR);
  Mat img_2 = imread(img2_path, cv::IMREAD_COLOR);
  
  if (img_1.data == nullptr || img_2.data == nullptr) {
    cerr << "Error reading images!" << endl;
    return;
  }

  //-- 初始化
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  //-- 第一步:检测 Oriented FAST 角点位置
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "Extract ORB cost = " << time_used.count() << " seconds." << endl;
  cout << "Image 1: " << keypoints_1.size() << " keypoints" << endl;
  cout << "Image 2: " << keypoints_2.size() << " keypoints" << endl;

  //-- 绘制并保存关键点
  Mat img_keypoints_1, img_keypoints_2;
  drawKeypoints(img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  drawKeypoints(img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> matches;
  t1 = chrono::steady_clock::now();
  matcher->match(descriptors_1, descriptors_2, matches);
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "Match ORB cost = " << time_used.count() << " seconds." << endl;

  //-- 第四步:匹配点对筛选
  // 计算最小距离和最大距离
  auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;

  printf("-- Max dist: %f \n", max_dist);
  printf("-- Min dist: %f \n", min_dist);

  //当描述子之间的距离大于2.5倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值50作为下限.
  std::vector<DMatch> good_matches;
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (matches[i].distance <= max(2.5 * min_dist, 35.0)) {
      good_matches.push_back(matches[i]);
    }
  }
  
  cout << "Good matches: " << good_matches.size() << "/" << matches.size() << endl;

  //-- 第五步:绘制匹配结果
  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
  
  // 创建输出目录
  filesystem::path output_dir(output_path);
  if (!filesystem::exists(output_dir)) {
    filesystem::create_directories(output_dir);
  }
  
  // 获取图像名称用于输出文件命名
  string img1_name = filesystem::path(img1_path).filename().stem().string();
  string img2_name = filesystem::path(img2_path).filename().stem().string();
  
  // 保存结果图像 - 每个图像的关键点
  string keypoints1_path = output_path + "/keypoints_" + img1_name + ".png";
  string keypoints2_path = output_path + "/keypoints_" + img2_name + ".png";
  imwrite(keypoints1_path, img_keypoints_1);
  imwrite(keypoints2_path, img_keypoints_2);
  
  // 保存结果图像 - 匹配结果
  string matches_path = output_path + "/matches_" + img1_name + "_" + img2_name + ".png";
  string goodmatches_path = output_path + "/goodmatches_" + img1_name + "_" + img2_name + ".png";
  
  imwrite(matches_path, img_match);
  imwrite(goodmatches_path, img_goodmatch);
  
  cout << "Saved keypoints to " << keypoints1_path << " and " << keypoints2_path << endl;
  cout << "Saved matches to " << matches_path << " and " << goodmatches_path << endl;
  cout << "----------------------------------------" << endl;
}

int main(int argc, char **argv) {
  string image_dir = "./images";
  string output_dir = "./output_cv";
  
  // 创建指定的图像对
  vector<pair<string, string>> image_pairs = {
    {image_dir + "/1.png", image_dir + "/2.png"},
    {image_dir + "/3.png", image_dir + "/4.png"},
    {image_dir + "/5.png", image_dir + "/6.png"}
  };
  
  // 处理指定的图像对
  for (const auto& pair : image_pairs) {
    process_image_pair(pair.first, pair.second, output_dir);
  }
  
  cout << "All specified image pairs processed." << endl;
  return 0;
}
