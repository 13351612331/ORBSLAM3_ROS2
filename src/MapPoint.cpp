//
// Created by kangyu on 2023/8/20.
//
#include "MapPoint.h"
#include <iostream>

namespace ORB_SLAM3 {
MapPoint::MapPoint() : mbBad(false), mpReplaced(static_cast<MapPoint *>(NULL)) {
  mpReplaced = static_cast<MapPoint *>(NULL);
}
} // namespace ORB_SLAM3