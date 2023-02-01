//
// Created by kangyu on 23-1-9.
//
#include <iostream>
#include <vector>
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
  std::cout << strFile << std::endl;
}