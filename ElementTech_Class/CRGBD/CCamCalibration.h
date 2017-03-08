#ifndef CCAMCALIBRATION_H
#define CCAMCALIBRATION_H

#include "../../opencv_header.h"

#include <iostream>

using namespace std;
//using namespace cv;

class CCAMCalibration {
public:
    CCAMCalibration() : m_flag(0), m_must_init_undistort(true) {}
    ~CCAMCalibration() {}

private:
    // the points in world coordinates
    std::vector<std::vector<cv::Point3f> > m_object_points;

    // the point positions in pixels
    std::vector<std::vector<cv::Point2f> > m_image_points;

    // flag to specify how calibration is done
    int m_flag;

    // used in image undistortion
    cv::Mat m_map1, m_map2;
    bool m_must_init_undistort;

public:

    // output Matrices
    cv::Mat m_camera_matrix;
    cv::Mat m_dist_coeffs;

    int AddChessboardPoints(const std::vector<std::string>& _filelist, cv::Size& _board_size);
    void AddPoints(const std::vector<cv::Point2f>& _image_corners, const std::vector<cv::Point3f>& _object_corners);
    double Calibrate(cv::Size &_image_size);
    cv::Mat ReMap(const cv::Mat& _image);
};





#endif // CCAMCALIBRATION_H
