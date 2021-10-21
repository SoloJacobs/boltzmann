#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

namespace fs = std::filesystem;

int main() {
  cv::VideoWriter output;

  std::string path{"/home/sol/cpp_basics/boltzmann/build/test_image.png"};
//cv::Mat inImg = cv::imread(path, cv::IMREAD_GRAYSCALE);
 // auto size = inImg.size();
  cv::Mat inImg(cv::Size(100,100),CV_8UC1);
  output.open("live1.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 15.0,
              inImg.size(), false);
  for (int i{0}; i < 100; ++i) {
    output.write(inImg);
  }
  size_t i{0};
  for (auto it = inImg.begin<unsigned char>(); it != inImg.end<unsigned char>();
       ++it) {
	  ++i;
    *it = i;
  }
  for (int i{0}; i < 100; ++i) {
	  output << inImg;
  }
}
