/* ************************************************* ellipse_detection.c *** *
 * 楕円検出クラス
 * ************************************************************************* */
#include <opencv2/features2d/features2d.hpp>
#include "ellipse_detection.h"
#include "ellipse_fitting.h"
#include "ellipse.h"
#include <algorithm>
#include <iterator>

/*
 * コンストラク
 */
EllipseDetection::EllipseDetection() {
  minLength          = DEFAULT_MIN_LENGTH;
  gaussianKernelSize = DEFAULT_GAUSSIAN_KERNEL_SIZE;
  gaussianSigma      = DEFAULT_GAUSSIAN_SIGMA;
  cannyParam[0]      = DEFAULT_CANNY_PARAM1;
  cannyParam[1]      = DEFAULT_CANNY_PARAM2;
  axisRatio          = DEFAULT_AXIS_RATIO;
  axisLength         = DEFAULT_AXIS_LENGTH;
  errorThreshold     = DEFAULT_ERROR_THRESHOLD;
  drawEllipseCenter  = false;
  ellipse_fitting.computeError = true;
}

/*
 * デストラクタ
 */
EllipseDetection::~EllipseDetection() {
  ;
}

static bool compareEllipseSize(const Ellips& e1, const Ellips& e2)
{ 
  return (e1.majorLength > e2.majorLength);
}

/*
 * 楕円検出関数
 *
 * @param [in] image         : 入力画像
 * @param [out] ellipse_list : 検出した楕円のリスト
 *
 * @return 楕円が２つ以上検出された場合はtrue, そうでなければfalse
 */
bool EllipseDetection::Detect(cv::Mat& 			image,
			      std::vector<Ellips>&	ellipse_list) {
  // 濃淡画像への変換
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);

  // 平滑化処理
  cv::Mat edge;
  cv::GaussianBlur(gray, edge,
		   cv::Size(gaussianKernelSize, gaussianKernelSize),
		   gaussianSigma);

  // エッジ検出
  cv::Canny(edge, edge, cannyParam[0], cannyParam[1]);

  // 輪郭線抽出
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edge, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  // 条件を満たす楕円が当てはまる点列をリストに追加
  std::vector<Ellips> candidate_list;
  for (int n = 0; n < contours.size(); n++) {

    // 点列数が閾値(minLength)より少ない場合は処理しない
    if (contours[n].size() < minLength) continue; 

    // 楕円当てはめ
    bool result = ellipse_fitting.Fit(contours[n]);

    // 条件を満たす楕円が当てはまる点列をリストに追加
    if (result && ellipse_fitting.error < errorThreshold) {
      Ellips ell(ellipse_fitting.u);
      ell.SetPoints(contours[n]);
      ell.ComputeAttributes();

      // 条件を満たすものをリストに追加
      if (ell.minorLength / ell.majorLength >= axisRatio &&
	  ell.majorLength > axisLength) { // 楕円リストに追加する条件
	
	      candidate_list.push_back(ell);
      }
    }
  }
  if (candidate_list.size() <= 1) return false;

  // 当てはめた楕円の長軸が長い順にソート
  std::sort(candidate_list.begin(), candidate_list.end(), compareEllipseSize);

  // 中心の近い楕円を統合
  Eigen::VectorXi use_index = Eigen::VectorXi::Ones(candidate_list.size());
  for (int n = 0; n < candidate_list.size() - 1; n++) {
    if (use_index(n) == 0) continue;

    Ellips target = candidate_list[n];

    for (int m = n + 1; m < candidate_list.size(); m++) {
      if (use_index(m) == 0) continue;

      Ellips reff = candidate_list[m];
      double dx = target.cx - reff.cx;
      double dy = target.cy - reff.cy;

      if ((dx * dx + dy * dy) < 4) {
	      use_index(m) = 0;
      }
    }
  }
  ellipse_list.clear();
  for (int n = 0; n < candidate_list.size(); n++) {
    if (use_index(n) == 1) {
      ellipse_list.push_back(candidate_list[n]);
    }
  }

  // 検出した楕円中心を描画
  //
  // 計算時間を短縮するためには以下の描画処理はコメントアウトした方がよい
  if (drawEllipseCenter) {
    for (int n = 0; n < ellipse_list.size(); n++) {
      Ellips ell = ellipse_list[n];
      cv::Point p;
      p.x = ell.cx;
      p.y = ell.cy;
      cv::circle(image, p, 3, cv::Scalar(0, 255, 0), 1, 1);
    }
  }
  return true;
}

/* ****************************************** End of ellipse_detection.c *** */
