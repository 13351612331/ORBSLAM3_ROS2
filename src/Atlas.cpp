//
// Created by kangyu on 23-5-30.
//
#include "Atlas.h"

namespace ORB_SLAM3 {
Atlas::Atlas() { mpCurrentMap = static_cast<Map *>(NULL); }
Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid) {
  mpCurrentMap = static_cast<Map *>(NULL);
}
Atlas::~Atlas() {}
} // namespace ORB_SLAM3