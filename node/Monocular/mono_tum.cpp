//
// Created by kangyu on 23-1-9.
//
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

#include "System.h"

void LoadImages(const std::string &strFile,
                std::vector<std::string> &vstrImageFilename,
                std::vector<double> &vTimestamps);

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << std::endl
              << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
                 "path_to_sequence"
              << std::endl;
    return 1;
  }

  // Retrieve paths to images
  std::vector<std::string> vstrImageFilenames{};
  std::vector<double> vTimestamps{};
  std::string strFile = std::string(argv[3]) + "/rgb.txt";
  LoadImages(strFile, vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::eSensor::MONOCULAR, true);
  float imageScale = SLAM.GetImageScale();

  // vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  std::cout << std::endl << "-------" << std::endl;
  std::cout << "Start processing sequence ..." << std::endl;
  std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

  double t_resize = 0.f;
  double t_track = 0.f;

  // Main loop
  cv::Mat im;
  for (int ni = 0; ni < nImages; ni++) {
    // Read image from file
    // cv::IMREAD_UNCHANGED
    // 指定了读取图片时不对其进行任何修改，也就是完整地读取并存储原始图像数据
    im = cv::imread(std::string(argv[3]) + "/" + vstrImageFilenames[ni],
                    cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];
    if (im.empty()) {
      std::cerr << std::endl
                << "Failed to load image at: "
                << std::string(argv[3]) + "/" + vstrImageFilenames[ni]
                << std::endl;
      return 1;
    }

    if (imageScale != 1.f) {
      std::cerr << "mono_tum: imageScale is not equal 1.f , please scale image"
                << std::endl;
    }

    SLAM.TrackMonocular(im, tframe);
  }
}

void LoadImages(const std::string &strFile,
                std::vector<std::string> &vstrImageFilename,
                std::vector<double> &vTimestamps) {
  std::ifstream f;
  f.open(strFile.c_str());

  // skip first three lines
  std::string s0;
  getline(f, s0);
  getline(f, s0);
  getline(f, s0);

  double t{};
  std::string sRGB{};

  while (!f.eof()) {
    std::string s;
    getline(f, s);
    if (!s.empty()) {
      // used to type convert
      std::stringstream ss;
      ss << s;
      // convert string to float
      ss >> t;
      vTimestamps.emplace_back(t);
      ss >> sRGB;
      vstrImageFilename.emplace_back(sRGB);
    }
  }
}