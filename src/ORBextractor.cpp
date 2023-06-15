//
// Created by kangyu on 23-6-15.
//
#include "ORBextractor.h"
#include <iostream>

namespace ORB_SLAM3 {
ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                           int _iniThFAST, int _minThFAST)
    : nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
      iniThFAST(_iniThFAST), minThFAST(_minThFAST) {
  mvScaleFactor.resize(nlevels);
  mvLevelSigma2.resize(nlevels);
  mvScaleFactor[0] = 1.0f;
  mvLevelSigma2[0] = 1.0f;

  for (int i = 1; i < nlevels; i++) {
    mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
    mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
  }

  mvInvScaleFactor.resize(nlevels);
  mvInvLevelSigma2.resize(nlevels);
  for (int i = 0; i < nlevels; i++) {
    mvInvScaleFactor[i] = 1.0 / mvScaleFactor[i];
    mvInvLevelSigma2[i] = 1.0 / mvInvLevelSigma2[i];
  }

  mvImagePyramid.resize(nlevels);

  mnFeaturesPerLevel.resize(nlevels);
  float factor = 1.0 / scaleFactor;
  float nDesiredFeaturesPerScale =
      nfeatures * (1 - factor) /
      (1 - (float)pow((double)factor, (double)nlevels));

  int sumFeatures = 0;
  for (int level = 0; level < nlevels - 1; level++) {
    mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
    sumFeatures += mnFeaturesPerLevel[level];
    nDesiredFeaturesPerScale *= factor;
  }
  mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);

  std::cout << "ORB Extractor TODO" << std::endl;
}
} // namespace ORB_SLAM3