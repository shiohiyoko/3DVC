/* *********************************************************** Rectanglee.c *** *
 * 楕円クラス
 * ************************************************************************* */
#include "Rectangle.h"
#include <algorithm>
#include <iterator>

/*
 * コンストラクタ
 */
Rectangle::Rectangle() {
  u  = Eigen::VectorXd::Zero(6);
  Q  = Eigen::Matrix3d::Zero(3, 3);
  cx = 0.0;
  cy = 0.0;
  majorLength = 0.0;
  minorLength = 0.0;

  for(int i=0; i<4; i++){
    x[i] = 0.0;
    y[i] = 0.0;
  }
}

/*
 * コンストラクタ
 *
 * @param [in] _u : 楕円パラメータ
 */
Rectangle::Rectangle(const Eigen::VectorXd& _u) {
  u  = _u;
  Q  = Eigen::Matrix3d::Zero(3, 3);
  Q << u(0), u(1), u(3), u(1), u(2), u(4), u(3), u(4), u(5);
  cx = 0.0;
  cy = 0.0;
  majorLength = 0.0;
  minorLength = 0.0;
  for(int i=0; i<4; i++){
    x[i] = 0.0;
    y[i] = 0.0;
  }
}

/*
 * デストラクタ
 */
Rectangle::~Rectangle() {
  ;
}

/*
 * 楕円パラメータを登録する関数
 *
 * @param [in] _u : 楕円パラメータ
 */
void Rectangle::SetParam(const Eigen::VectorXd& _u) {
  u  = _u;
  Q << u(0), u(1), u(3), u(1), u(2), u(4), u(3), u(4), u(5);
}

/*
 * 点列を登録する関数
 *
 * @param [in] _points : 点のリスト
 */
void Rectangle::SetPoints(std::vector <cv::Point> _points) {
  std::copy(_points.begin(), _points.end(), std::back_inserter(point_list));
}

/*
 * 楕円属性(楕円中心の座標, 長軸と短軸の長さ)を計算する関数
 */
void Rectangle::ComputeAttributes(void) {
  double A = u(0);
  double B = u(1);
  double C = u(2);
  double D = u(3);
  double E = u(4);
  double F = u(5);

  cx = -(C * D - B * E) / (A * C - B * B);
  cy = -(A * E - B * D) / (A * C - B * B);

  double c = A * cx * cx + 2 * B * cx * cy + C * cy * cy - F;

  A /= c;
  B /= c;
  C /= c;

  double part = sqrt((A + C) * (A + C) - 4 * (A * C - B * B));
  double lambda1 = 0.5 * ((A + C) - part);
  double lambda2 = 0.5 * ((A + C) + part);

  majorLength = 1.0 / sqrt(fabs(lambda1));
  minorLength = 1.0 / sqrt(fabs(lambda2));
}

/*
 * 楕円パラメータに点(x, y)を代入した値を計算する関数
 *
 * @param [in] x, y : 点(x, y)
 *
 * @return 楕円パラメータに点(x, y)を代入した値
 */
double Rectangle::RectangleeValue(double x, double y) {
  double A = u(0);
  double B = u(1);
  double C = u(2);
  double D = u(3);
  double E = u(4);
  double F = u(5);

  return A * x * x + 2 * B * x * y + C * y * y + 2 * D * x + 2 * E * y + F;
}

/*
 * 点(x, y)が楕円の外部か内部かを判定する関数 
 *
 * @param [in] x, y: 点(x, y)
 *
 * @return 点(x, y)が楕円の内部であればtrue, そうでなければfalse
 */
bool Rectangle::CheckInner(double x, double y) {
  static bool initialized = false;
  static double center_sgn = 0;

  if (!initialized) {
    center_sgn = RectangleeValue(cx, cy);
    initialized = true;
  }
  double target_sgn = RectangleeValue(x, y);

  bool retval = center_sgn * target_sgn > 0 ? true : false;

  return retval;
}

void CircularMarker::ComputeCameraParam(void) {
  // 指示平面の法線ベクトルとカメラから指示平面までの距離を計算
  Eigen::VectorXd normalOuter(3), normalInner(3);
  double distOuter = ComputeNormal (ellipseOuter.Q,
				    normalOuter, radiusOuter, position);

  // 世界座標系のＹ軸の計算
  Eigen::Vector3d Y = normalOuter;

  // カメラから支持平面までの距離の計算
  double dist = distOuter;
  
  // 外側の円の中心を指すベクトルを計算
  Eigen::VectorXd XcOuter = ellipseOuter.Q.inverse() * Y;
  XcOuter(0) /= XcOuter(2);
  XcOuter(1) /= XcOuter(2);
  XcOuter(2) = 1.0;
  Eigen::VectorXd RcOuter = -dist * XcOuter / Y.dot(XcOuter);

  // 内側の円の中心を指すベクトルを計算
  Eigen::VectorXd XcInner = ellipseInner.Q.inverse() * Y;
  XcInner(0) /= XcInner(2);
  XcInner(1) /= XcInner(2);
  XcInner(2) = 1.0;
  Eigen::VectorXd RcInner = -dist * XcInner / Y.dot(XcInner);

  // カメラの並進ベクトルの生成
  T = RcOuter;

  // 世界座標系のＺ軸を計算（二つの円の中心を結んだ方向）
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Vector3d Z = (((I - Y * Y.transpose())
			* (RcInner - RcOuter)).normalized());

  // 世界座標系のＸ軸を計算（Ｙ軸とＺ軸の外積として計算）
  Eigen::Vector3d X = Y.cross(Z).normalized();

  // カメラの回転行列を生成
  R.col(0) = X;
  R.col(1) = Y;
  R.col(2) = Z;

  // 座標系の変換行列
  Eigen::MatrixXd R_ = Eigen::Matrix3d::Zero(3, 3);
  R_(0, 1) = 1.0;
  R_(1, 0) = 1.0;
  R_(2, 2) = -1.0;

  // 座標系の変換
  R = R_ * R;
  T = R_ * T;
	   
  // モデルビュー行列の生成
  M[0] = R(0, 0); M[1] = R(1, 0); M[2]  = R(2, 0);
  M[4] = R(0, 1); M[5] = R(1, 1); M[6]  = R(2, 1);
  M[8] = R(0, 2); M[9] = R(1, 2); M[10] = R(2, 2);
  M[12] = T(0); M[13] = T(1); M[14] = T(2);
}
/* **************************************************** End of Rectanglee.c *** */
