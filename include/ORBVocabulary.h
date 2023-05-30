//
// Created by kangyu on 23-5-28.
//

#ifndef ORB_SLAM3_ORBVOCABULARY_H
#define ORB_SLAM3_ORBVOCABULARY_H

/**
 * 用于ORB特征描述子聚类的一个工具类，它提供了一些API来实现ORB特征描述子的带权重代码本的构建、保存和读取等功能
 */
#include "DBoW2/DBoW2/FORB.h"
/**
 * 使得用户可以使用不同的描述子类型（SIFT、SURF、ORB等）来构建词袋模型
 * 需要根据实际情况选择合适的描述子类型，并设置聚类中心数量和其他参数
 */
#include "DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM3 {
/**
 * DBoW2::FORB::TDescriptor 表示ORB特征的描述子类型
 * DBoW2::FORB ORB特征的描述子类
 */
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_ORBVOCABULARY_H
