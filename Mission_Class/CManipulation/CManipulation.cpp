#include "CManipulation.h"

CManipulation::CManipulation(){

}

CManipulation::CManipulation(CLRF *_p_lrf, CCamera *_p_camera, CKinova *_p_kinova, CVehicle *_p_vehicle, CVelodyne *_p_velodyne){

    mpc_lrf = _p_lrf;
    mpc_camera = _p_camera;
    mpc_kinova = _p_kinova;
    mpc_vehicle = _p_vehicle;
    mpc_velodyne = _p_velodyne;

    mpc_rgb_d = new CRGBD(_p_camera, _p_lrf);

    connect(mpc_kinova, SIGNAL(SignalKinovaPosition(CartesianPosition)), this, SIGNAL(SignalKinovaPosition(CartesianPosition)));
    connect(mpc_lrf, SIGNAL(SignalLRFImage(cv::Mat)), this, SIGNAL(SignalLRFImage(cv::Mat)));
    connect(mpc_camera, SIGNAL(SignalCameraImage(cv::Mat)), this, SIGNAL(SignalCameraImage(cv::Mat)));

    connect(mpc_camera, SIGNAL(SignalCameraImage(cv::Mat)), this, SIGNAL(SignalCameraImage(cv::Mat)));
    connect(mpc_rgb_d, SIGNAL(SignalSegnetImage(cv::Mat)), this, SIGNAL(SignalSegnetImage(cv::Mat)));
}


//Kinova
bool CManipulation::InitKinova(){

    if(mpc_kinova->IsKinovaInitialized())
        return true;
    else{
        if(mpc_kinova->InitKinova())
            return true;
        else
            return false;
    }
}

bool CManipulation::CloseKinova(){

    mpc_kinova->CloseKinova();

    return true;
}

bool CManipulation::KinovaInitMotion(){

    if(!InitKinova())
        return false;

    mpc_kinova->KinovaInitMotion();

    return true;
}

bool CManipulation::KinovaAlignPanel(){

    if(!InitKinova())
        return false;

    if(!mpc_kinova->IsKinovaInitPosition())
        if(!KinovaInitMotion())
            return false;

    mpc_kinova->KinovaAlignToPanel();

    return true;
}

bool CManipulation::KinovaDoManipulate(CartesianPosition _position){

    if(!mpc_kinova->KinovaDoManipulate(_position))
        return false;

    return true;
}

bool CManipulation::KinovaMoveUnitStepUp(){

    if(!mpc_kinova->KinovaMoveUnitStepUp())
        return false;

    return true;
}

bool CManipulation::KinovaMoveUnitStepDw(){

    if(!mpc_kinova->KinovaMoveUnitStepDw())
        return false;

    return true;
}

bool CManipulation::KinovaMoveUnitStepRi(){

    if(!mpc_kinova->KinovaMoveUnitStepRi())
        return false;

    return true;
}

bool CManipulation::KinovaMoveUnitStepLe(){

    if(!mpc_kinova->KinovaMoveUnitStepLe())
        return false;

    return true;
}

bool CManipulation::KinovaGetPosition(CartesianPosition& _position){

    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    _position = mpc_kinova->KinovaGetPosition();

    return true;
}

//LRF
bool CManipulation::InitLRF(){

    if(!mpc_lrf->InitLRF())
        return false;

    return true;
}

bool CManipulation::CloseLRF(){

    mpc_lrf->CloseLRF();

    return true;
}

bool CManipulation::IsLRFConnected(){

    return mpc_lrf->isRunning();
}


//Camera
bool CManipulation::InitCamera(){

    if(!mpc_camera->InitCamera(CAMERA_DEVICE_NUMBER))
        return false;

    return true;
}

void CManipulation::CloseCamera(){
    mpc_camera->CloseCamera();
}

bool CManipulation::IsCameraConnected(){
    return mpc_camera->IsCameraConnected();
}

bool CManipulation::SetRGBDFunction(int _index){

    if(!mpc_rgb_d->RGB_DThreadSetting(_index))
        return false;

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CManipulation::run(){


}

