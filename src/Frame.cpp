//
// Created by kangyu on 23-7-2.
//
#include "Frame.h"

namespace ORB_SLAM3 {
long unsigned int Frame::nNextId = 0;
bool Frame::mbInitialComputations = true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

// 单目模式
Frame::Frame(const cv::Mat &imGray, const double &timeStamp,
             ORB_SLAM3::ORBextractor *extractor, ORB_SLAM3::ORBVocabulary *voc,
             ORB_SLAM3::GeometricCamera *pCamera, cv::Mat &distCoef,
             const float &bf, const float &thDepth, ORB_SLAM3::Frame *pPrevF,
             const IMU::Calib &ImuCalib)
    : mpORBextractorLeft(extractor),
      mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
      mDistCoef(distCoef.clone()), mpCamera(pCamera), mpCamera2(nullptr),
      mK(static_cast<Pinhole *>(pCamera)->toK()), mbf(bf) {
  // Frame ID
  // Step 1 帧的ID 自增
  mnId = nNextId++;

  // Step 2 计算图像金字塔的参数
  // Scale Level Info
  // 获取图像金字塔的层数
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  // 获取每层的缩放因子
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  // 计算每层缩放因子的自然对数
  mfLogScaleFactor = log(mfScaleFactor);
  // 获取各层图像的缩放因子
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  // 获取各层图像的缩放因子的倒数
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  // 获取sigma^2
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  // 获取sigma^2的倒数
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  // Step 3 对这个单目图像进行提取特征点, 第一个参数0-左图， 1-右图
  ExtractORB(0, imGray, 0, 1000);

  // 提取特征点的个数
  N = mvKeys.size();
  // 如果没有能够成功提取出特征点，那么就直接返回了
  if (mvKeys.empty())
    return;

  // Step 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
  UndistortKeyPoints();

  // Set no stereo information
  // 由于单目相机无法直接获得立体信息，所以这里要给右图像对应点和深度赋值-1表示没有相关信息
  mvuRight = vector<float>(N, -1);
  mvDepth = vector<float>(N, -1);
  mnCloseMPs = 0;

  // 初始化本帧的地图点
  mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

  mmProjectPoints.clear();
  mmMatchedInImage.clear();

  mvbOutlier = vector<bool>(N, false);

  // This is done only for the first Frame (or after a change in the
  // calibration)
  //  Step 5
  //  计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
  if (mbInitialComputations) {
    // 计算去畸变后图像的边界
    ComputeImageBounds(imGray);
    // 表示一个图像像素相当于多少个图像网格列（宽）
    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) /
                            static_cast<float>(mnMaxX - mnMinX);
    // 表示一个图像像素相当于多少个图像网格行（高）
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) /
                             static_cast<float>(mnMaxY - mnMinY);

    fx = static_cast<Pinhole *>(mpCamera)->toK().at<float>(0, 0);
    fy = static_cast<Pinhole *>(mpCamera)->toK().at<float>(1, 1);
    cx = static_cast<Pinhole *>(mpCamera)->toK().at<float>(0, 2);
    cy = static_cast<Pinhole *>(mpCamera)->toK().at<float>(1, 2);

    invfx = 1.0 / fx;
    invfy = 1.0 / fy;

    // 特殊的初始化过程完成，标志复位
    mbInitialComputations = false;
  }

  // 计算 basline
  mb = mbf / fx;

  // Set no stereo fisheye information
  Nleft = -1;
  Nright = -1;
  mvLeftToRightMatch = vector<int>(0);
  mvRightToLeftMatch = vector<int>(0);
  mTlr = cv::Mat(3, 4, CV_32F);
  mTrl = cv::Mat(3, 4, CV_32F);
  mvStereo3Dpoints = vector<cv::Mat>(0);
  monoLeft = -1;
  monoRight = -1;

  // 将特征点分配到图像网格中
  AssignFeaturesToGrid();
}

void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0,
                       const int x1) {
  vector<int> vLapping = {x0, x1};
  if (flag == 0)
    monoLeft =
        (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
  else
    std::cout << "[Frame.cpp::ExtractORB]: 双目未实现" << std::endl;
}

/**
 * @brief 用内参对特征点去畸变，结果报存在mvKeysUn中
 *
 */
void Frame::UndistortKeyPoints() {
  // Step 1
  // 如果第一个畸变参数为0，不需要矫正。第一个畸变参数k1是最重要的，一般不为0，为0的话，说明畸变参数都是0
  // 变量mDistCoef中存储了opencv指定格式的去畸变参数，格式为：(k1,k2,p1,p2,k3)
  if (mDistCoef.at<float>(0) == 0.0) {
    mvKeysUn = mvKeys;
    return;
  }

  // Fill matrix with points
  // Step 2 如果畸变参数不为0，用OpenCV函数进行畸变矫正
  // N为提取的特征点数量，为满足OpenCV函数输入要求，将N个特征点保存在N*2的矩阵中
  cv::Mat mat(N, 2, CV_32F);
  // 遍历每个特征点，并将它们的坐标保存到矩阵中
  for (int i = 0; i < N; i++) {
    // 然后将这个特征点的横纵坐标分别保存
    mat.at<float>(i, 0) = mvKeys[i].pt.x;
    mat.at<float>(i, 1) = mvKeys[i].pt.y;
  }

  // Undistort points
  // 函数reshape(int cn,int rows=0)
  // 其中cn为更改后的通道数，rows=0表示这个行将保持原来的参数不变
  // 为了能够直接调用opencv的函数来去畸变，需要先将矩阵调整为2通道（对应坐标x,y）
  // cv::undistortPoints最后一个矩阵为空矩阵时，得到的点为归一化坐标点
  mat = mat.reshape(2);
  cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(),
                      mDistCoef, cv::Mat(), mK);
  // 调整回只有一个通道，回归我们正常的处理方式
  mat = mat.reshape(1);

  // Fill undistorted keypoint vector
  // Step 3 存储校正后的特征点
  mvKeysUn.resize(N);
  for (int i = 0; i < N; i++) {
    // 根据索引获取这个特征点
    // 注意之所以这样做而不是直接重新声明一个特征点对象的目的是，能够得到源特征点对象的其他属性
    cv::KeyPoint kp = mvKeys[i];
    // 读取校正后的坐标并覆盖老坐标
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    mvKeysUn[i] = kp;
  }
}

/**
 * @brief 计算去畸变图像的边界
 *
 * @param[in] imLeft            需要计算边界的图像
 */
void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
  // 如果畸变参数不为0，用OpenCV函数进行畸变矫正
  if (mDistCoef.at<float>(0) != 0.0) {
    // 保存矫正前的图像四个边界点坐标： (0,0) (cols,0) (0,rows) (cols,rows)
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;

    mat.at<float>(1, 0) = imLeft.cols;
    mat.at<float>(1, 1) = 0.0;

    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imLeft.rows;

    mat.at<float>(3, 0) = imLeft.cols;
    mat.at<float>(3, 1) = imLeft.rows;

    // 和前面校正特征点一样的操作，将这几个边界点作为输入进行校正
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(),
                        mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    // Undistort corners
    // 校正后的四个边界点已经不能够围成一个严格的矩形，因此在这个四边形的外侧加边框作为坐标的边界
    mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  } else {
    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
  }
}

/**
 * @brief 特征分网格
 */
void Frame::AssignFeaturesToGrid() {
  // Fill matrix with points
  // Step 1  给存储特征点的网格数组 Frame::mGrid 预分配空间
  const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

  int nReserve = 0.5 * N / (nCells);

  // 开始对mGrid这个二维数组中的每一个vector元素遍历并预分配空间
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
      mGrid[i][j].reserve(nReserve);
      if (Nleft != -1) {
        std::cerr << "[Frame.cpp:AssignFeaturesToGrid]: 1.Not implement"
                  << std::endl;
      }
    }

  // Step 2
  // 遍历每个特征点，将每个特征点在mvKeysUn中的索引值放到对应的网格mGrid中
  for (int i = 0; i < N; i++) {
    const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                             : (i < Nleft) ? mvKeys[i]
                                           : mvKeysRight[i - Nleft];

    // 存储某个特征点所在网格的网格坐标，nGridPosX范围：[0,FRAME_GRID_COLS],
    // nGridPosY范围：[0,FRAME_GRID_ROWS]
    int nGridPosX, nGridPosY;
    // 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
    if (PosInGrid(kp, nGridPosX, nGridPosY)) {
      if (Nleft == -1 || i < Nleft)
        mGrid[nGridPosX][nGridPosY].push_back(i);
      else
        std::cerr << "[Frame.cpp:AssignFeaturesToGrid]: 2.Not implement"
                  << std::endl;
    }
  }
}

/**
 * @brief
 * 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
 *
 * @param[in] kp                    给定的特征点
 * @param[in & out] posX            特征点所在网格坐标的横坐标
 * @param[in & out] posY            特征点所在网格坐标的纵坐标
 * @return true                     如果找到特征点所在的网格坐标，返回true
 * @return false                    没找到返回false
 */
bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
  // 计算特征点x,y坐标落在哪个网格内，网格坐标为posX，posY
  posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
  posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the
  // image
  //  因为特征点进行了去畸变，而且前面计算是round取整，所以有可能得到的点落在图像网格坐标外面
  //  如果网格坐标posX，posY超出了[0,FRAME_GRID_COLS]
  //  和[0,FRAME_GRID_ROWS]，表示该特征点没有对应网格坐标，返回false
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 ||
      posY >= FRAME_GRID_ROWS)
    return false;

  return true;
}
} // namespace ORB_SLAM3