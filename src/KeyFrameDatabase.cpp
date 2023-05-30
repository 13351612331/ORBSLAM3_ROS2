//
// Created by kangyu on 23-5-30.
//
#include "KeyFrameDatabase.h"
namespace ORB_SLAM3 {
KeyFrameDatabase::KeyFrameDatabase(const ORB_SLAM3::ORBVocabulary &voc)
    : mpVoc(&voc) {
  mvInvertedFile.resize(voc.size());
}
} // namespace ORB_SLAM3