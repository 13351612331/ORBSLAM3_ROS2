//
// Created by kangyu on 2023/8/20.
//

#ifndef ORB_SLAM3_MAPPOINT_H
#define ORB_SLAM3_MAPPOINT_H

namespace ORB_SLAM3 {
class MapPoint {
public:
  MapPoint();

protected:
  // Bad flag (we do not currently erase MapPoint from memory)
  // 表示地图点（MapPoint）是否是一个坏点的标志。它是一个布尔类型的变量，如果地图点被标记为坏点，则其值为true，否则为false。该标志可能用于指示地图点的质量或可用性，以便在后续的处理中进行相应的操作。
  bool mbBad;
  // 指向替换当前地图点的另一个地图点的指针。它是一个指向MapPoint对象的指针。当当前地图点被替换时，mpReplaced指向新的地图点对象，以保留替换之前的引用关系。这个变量可能在地图点更新或替换的过程中使用，以在需要时引用替换前的地图点。
  MapPoint *mpReplaced;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_MAPPOINT_H
