//
// Created by kangyu on 23-5-30.
//

#ifndef ORB_SLAM3_KEYFRAMEDATABASE_H
#define ORB_SLAM3_KEYFRAMEDATABASE_H
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include <eigen3/Eigen/Core>
#include <list>
#include <vector>

namespace ORB_SLAM3 {
class KeyFrameDatabase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KeyFrameDatabase() = default;
  KeyFrameDatabase(const ORBVocabulary &voc);

protected:
  // Associated vocabulary
  const ORBVocabulary *mpVoc;

  /**
   * std::vector<list<KeyFrame*>> mvInvertedFile 也是ORB-SLAM3中DBoW2库中的一个变量，
   * 它表示DBoW2词袋模型的倒排索引表（inverted file）。
   * 在ORB-SLAM2中，DBoW2库使用聚类ORB特征描述子来构建词袋模型，
   * 因此每个单词（word）都对应着一个ORB特征描述子的类簇（cluster）。
   * 而每个类簇又可以对应于多个包含该类簇的关键帧（KeyFrame）。
   * 为了快速地实现图像检索，DBoW2库使用倒排索引表，
   * 对于每个单词，倒排列表记录了包含该单词的所有关键帧的标识符和对应的出现次数。
   * mvInvertedFile是一个二维 vector，其中每个元素表示一个单词的倒排列表。
   * 具体来说，它是由 std::list<KeyFrame*> 类型组成的 vector，即一个 vector，
   * 其中的每个元素都是一个存储了 KeyFrame 指针的链表。每个链表的头结点代表一个单词，
   * 链表中的其他节点则代表含有这个单词的关键帧。
   * 因此，每个 KeyFrame 指针对应于一个包含该单词的关键帧。
   */
  std::vector<list<KeyFrame*>> mvInvertedFile;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_KEYFRAMEDATABASE_H
