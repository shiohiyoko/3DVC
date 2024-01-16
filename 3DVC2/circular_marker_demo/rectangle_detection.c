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
RectangleDetection::RectangleDetection() {
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
RectangleDetection::~RectangleDetection() {
  ;
}

/*
 * 楕円検出関数
 *
 * @param [in] image         : 入力画像
 * @param [out] ellipse_list : 検出した楕円のリスト
 *
 * @return 楕円が２つ以上検出された場合はtrue, そうでなければfalse
 */
bool RectangleDetection::Detect(const cv::Mat& image, 
            vector<vector<cv::Point> >& rect_list)
{
    squares.clear();
    
    Mat pyr, timg, cv::gray0(image.size(), CV_8U), gray;
    
    // down-scale and upscale the image to filter out the noise
    cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    cv::pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cv::Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    cv::fabs(cv::contourArea(approx)) > 1000 &&
                    cv::isContourConvex(approx) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = cv::fabs(cv::angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
    if (squares.size() > 0) return true;
    else return false;
}


/* ****************************************** End of ellipse_detection.c *** */
