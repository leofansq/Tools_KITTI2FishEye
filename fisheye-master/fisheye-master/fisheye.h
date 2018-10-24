#ifndef _FISHEYE_CALIBRATION_H
#define _FISHEYE_CALIBRATION_H

#include <opencv2/core/core.hpp>
using cv::Mat;
using cv::Point2f;

void PointMap(Point2f sp, Point2f &dp, float r);
void PointMap(float x, float y, float& new_x, float& new_y, float r);
void PointMap2(float x, float y, float& new_x, float& new_y, float r);
void RectifyMap(Mat& mapx, Mat& mapy, float r = 600);
void UndisImage(Mat distort_image, Mat& undistort_image, Mat mapx, Mat mapy);
void LabelPoint(float x, float y, float& new_x, float& new_y, float r);
void LabelTrans(float x1, float x2, float y1, float y2, float& bbx1, float& bbx2, float& bby1, float& bby2);

#endif
