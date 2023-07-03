//
// Created by kangyu on 23-7-2.
//
#include "Frame.h"

namespace ORB_SLAM3 {
long unsigned int Frame::nNextId = 0;
Frame::Frame(const cv::Mat &imGray, const double &timeStamp,
             ORB_SLAM3::ORBextractor *extractor, ORB_SLAM3::ORBVocabulary *voc,
             ORB_SLAM3::GeometricCamera *pCamera, cv::Mat &distCoef,
             const float &bf, const float &thDepth, ORB_SLAM3::Frame *pPrevF,
             const IMU::Calib &ImuCalib)
    : mpORBextractorLeft(extractor),
      mpORBextractorRight(static_cast<ORBextractor *>(NULL)) {
  // Frame ID
  mnId = nNextId++;

  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
}

void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1) {
  vector<int> vLapping = {x0 , x1};
  if(flag == 0)
//    monoLeft = (*mpORBextractorLeft)()
}
} // namespace ORB_SLAM3