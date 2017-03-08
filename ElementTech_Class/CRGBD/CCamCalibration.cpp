#include "CCamCalibration.h"

int CCAMCalibration::AddChessboardPoints(const std::vector<std::__cxx11::string> &_filelist, cv::Size &_board_size)
{
    // the points on the chessboard
    vector<cv::Point2f> image_corners;
    vector<cv::Point3f> object_corners;

    // 3D Scene points
    // Initialize the chessboard corners in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z) = (i,j,0)
    for (int i = 0; i < _board_size.height;i++)
    {
        for (int j = 0; j < _board_size.width;j++)
        {
            object_corners.push_back(cv::Point3f(i,j,0.0f));
        }
    }

    // 2D image points
    cv::Mat image;
    int successes = 0;
    for (int i=0; i < _filelist.size();i++)
    {
        // Open the image
        image = cv::imread(_filelist[i],0);

        // Get the chessboard corners
        bool found = cv::findChessboardCorners(image,_board_size,image_corners);

        if (found)
        {
            //Get subpixel accuracy on the corners
            cv::cornerSubPix(image, image_corners, cv::Size(11,11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));

            // If we have a good board, add it to our data
            if (image_corners.size() == _board_size.area())
            {
                AddPoints(image_corners,object_corners);
                successes++;
            }
        }
    }

    return successes;
}

void CCAMCalibration::AddPoints(const std::vector<cv::Point2f> &_image_corners, const std::vector<cv::Point3f> &_object_corners)
{
    // 2D image points from one view
    m_image_points.push_back(_image_corners);
    // correspoinding 3D scene points
    m_object_points.push_back(_object_corners);
}

double CCAMCalibration::Calibrate(cv::Size &_image_size)
{
    // undistorter must be reinitialized
    m_must_init_undistort = true;

    // Output rotations and translations
    cv::Matx33d K;
    cv::Vec4d D;
    vector<cv::Mat> rvecs, tvecs;

    double error = cv::calibrateCamera(m_object_points,
                                   m_image_points,
                                   _image_size,
                                   m_camera_matrix,
                                   m_dist_coeffs,
                                   rvecs, tvecs,
                                   m_flag);

    cv::FileStorage fs("CAM_param.xml",cv::FileStorage::WRITE);
    fs << "m_camera_matrix" << m_camera_matrix;
    fs << "m_dist_coeffs" << m_dist_coeffs;
    fs.release();

    return error;
}

cv::Mat CCAMCalibration::ReMap(const cv::Mat &_image)
{
    cv::Mat undistorted;

    cv::FileStorage fs("/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/Device_Class/CCamera/CAM_param.xml",cv::FileStorage::READ);
    fs["m_camera_matrix"] >> m_camera_matrix;
    fs["m_dist_coeffs"] >> m_dist_coeffs;
    fs.release();

    cv::Size size_remap;
    size_remap.width = _image.size().width*1.3;
    size_remap.height = _image.size().height*1.3;
//    size_remap = _image.size();

    if(m_must_init_undistort)
    {
        cv::initUndistortRectifyMap(
                    m_camera_matrix, // computed camera matrix
                    m_dist_coeffs,   // computed distortion matrix
                    cv::Mat(),       // optional rectification (none)s
                    cv::Mat(),       // camera matrix to generate undistorted
                    size_remap,      // size of undistorted
                    CV_32FC1,        // type of output map
                    m_map1, m_map2);     //

    }

    // Apply mapping function
    cv::remap(_image, undistorted, m_map1, m_map2, cv::INTER_LINEAR);

    return undistorted;
}
