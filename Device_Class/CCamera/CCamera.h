#ifndef CCAMERA_H
#define CCAMERA_H


#include <QtWidgets>
#include <QThread>

#include "../../opencv_header.h"

#include "ElementTech_Class/CSSD/CSSD.h"

#define CAMERA_DEVICE_NUMBER 1

class CCamera: public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CCamera();
    ~CCamera();

private:
    bool fl_stream;
    bool fl_camera_init;

    cv::Mat m_mat_original_image;
    cv::VideoCapture m_cam;

    QMutex mtx_camera;

public:
    bool InitCamera(std::string _ip_number);
    void CloseCamera();
    bool IsCameraConnected();

    bool GetCameraImage(cv::Mat &_image);

signals:
    void SignalCameraImage(cv::Mat);
};

#endif // CCAMERA_H
