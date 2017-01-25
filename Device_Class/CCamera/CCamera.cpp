#include "CCamera.h"

CCamera::CCamera(){
    fl_stream = false;
    fl_camera_init = false;
}

CCamera::~CCamera(){

    fl_stream = false;
    while(this->isRunning());

    CloseCamera();
}


bool CCamera::InitCamera(int _dev_number){

    if(fl_camera_init)
        return true;

    if(!m_cam.open(_dev_number))
        return false;

    else{
        fl_camera_init = true;
        fl_stream = true;

        this->start();

        return true;
    }
}

void CCamera::CloseCamera(){

    fl_stream = false;
    fl_camera_init = false;

    while(this->isRunning());

    mtx_camera.lock();
    {
        m_cam.release();
    }
    mtx_camera.unlock();

}

bool CCamera::IsCameraConnected(){
    return this->isRunning();
}

bool CCamera::GetCameraImage(cv::Mat &_image){

    mtx_camera.lock();
    {
         _image = m_mat_original_image.clone();
//       m_cam >> _image;
    }
    mtx_camera.unlock();
    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CCamera::run(){

    while(fl_stream){

        mtx_camera.lock();
        {
            m_cam >> m_mat_original_image;
            emit SignalCameraImage(m_mat_original_image);
        }
        mtx_camera.unlock();
        msleep(30);
    }
}
