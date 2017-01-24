#include "CRGBD.h"

CRGBD::CRGBD(){

}

CRGBD::CRGBD(CCamera* _pc_camera, CLRF* _pc_lrf){

    mpc_lrf = _pc_lrf;
    mpc_camera = _pc_camera;
}

CRGBD::~CRGBD(){
    while(this->isRunning());
}


//-------------------------------------------------
// Element Tech Function
//-------------------------------------------------

void CRGBD::GetLRFInfo(double &_slope, double &_distance, double _s_deg, double _e_deg){
    mpc_lrf->GetLRFData(mary_lrf_distance);

    mary_lrf_distance;
}

cv::Mat CRGBD::GetSegnetImage(cv::Mat _org_img){

    cv::Mat segnet_image;
    segnet_image = mc_segnet.GetSegnetImage(_org_img);

    return segnet_image;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CRGBD::run(){

}
