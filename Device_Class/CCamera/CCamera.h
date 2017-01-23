#ifndef CCAMERA_H
#define CCAMERA_H


#include <QtWidgets>
#include <QThread>

#include "../../opencv_header.h"

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
    bool InitCamera(int _dev_number);
    void CloseCamera();
    bool IsCameraConnected();

signals:
    void SignalCameraImage(cv::Mat);
};

#endif // CCAMERA_H
