//
// Created by kangyu on 23-1-9.
//
#include <fstream>
#include <iostream>
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