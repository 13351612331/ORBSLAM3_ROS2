//
// Created by kangyu on 23-6-1.
//
#include "FrameDrawer.h"

namespace ORB_SLAM3 {
FrameDrawer::FrameDrawer(ORB_SLAM3::Atlas *pAtlas)
    : both(false), mpAtlas(pAtlas) {
  mState = eTrackingState::SYSTEM_NOT_READY;
  /**
   * 480：图像的高度，即行数。
   * 640：图像的宽度，即列数。
   * CV_8UC3：数据类型，表示每个像素使用 8
   * 位无符号整数表示，每个像素有三个通道，分别表示蓝色、绿色和红色通道。
   * cv::Scalar(0, 0, 0)：初始化每个像素的值，表示所有像素的初始值都是 (0, 0,
   * 0)，即黑色。
   */
  mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  mImRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}
} // namespace ORB_SLAM3