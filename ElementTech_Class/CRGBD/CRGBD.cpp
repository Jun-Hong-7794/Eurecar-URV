#include "CRGBD.h"

CRGBD::CRGBD(){
    fl_function_index = 0;
}

CRGBD::CRGBD(CCamera* _pc_camera, CLRF* _pc_lrf){

    mpc_lrf = _pc_lrf;
    mpc_camera = _pc_camera;
}

CRGBD::~CRGBD(){
    while(this->isRunning());
}


bool CRGBD::RGB_DThreadSetting(int _function_index){

    if(this->isRunning()){
        return false;
    }

    fl_function_index = (_function_index);

    this->start();

    return true;
}

void CRGBD::SegnetFunction(){

    cv::Mat camera_image;
    cv::Mat segnet_image;
    while(fl_function_index == THREAD_SEGNET_INDEX){
        if(!mpc_camera->GetCameraImage(camera_image))
            return;
        segnet_image = mc_segnet.GetSegnetImage(camera_image);

        emit SignalSegnetImage(segnet_image);
    }

    return;
}

//-------------------------------------------------
// Element Tech Function
//-------------------------------------------------

void CRGBD::GetLRFInfo(double &_slope, double &_distance, double _s_deg, double _e_deg){

    if((_s_deg < 0 || _e_deg > 180) || (_s_deg >= _e_deg)){//Out Of Range . . .

        _slope = 0;
        _distance = 0;

        std::cout << "LRF Warning: Out of Range" << std::endl;
        return;
    }

    //Convert to Original Coordinate
    _s_deg += 45;
    _e_deg += 45;

    int s_index = (int)(_s_deg / ANGLE_RESOLUTION);
    int e_index = (int)(_e_deg / ANGLE_RESOLUTION);

    int number_of_point = e_index -s_index + 1;

    long* lrf_distance = new long[number_of_point];

    if(!mpc_lrf->GetLRFData(mary_lrf_distance)){
        std::cout << "LRF : GetLRFData Error" << std::endl;
        return;
    }
    else{
        memcpy(lrf_distance, &mary_lrf_distance[s_index], sizeof(long)*(number_of_point));
        ClaculateLRFHeightDistance(lrf_distance, _s_deg);
    }

    delete[] lrf_distance;

    return;
}

cv::Mat CRGBD::GetSegnetImage(cv::Mat _org_img){

    cv::Mat segnet_image;
    segnet_image = mc_segnet.GetSegnetImage(_org_img);

    return segnet_image;
}

//-------------------------------------------------
// Calculation Function
//-------------------------------------------------

void CRGBD::ClaculateLRFHeightDistance(long* _lrf_org_data, double _s_deg){

    for(int i = 0; i < sizeof(_lrf_org_data); i++){
        POINT_PARAM point_parameter;

        double deg = i*ANGLE_RESOLUTION + _s_deg;

        if(deg > 90){// X axis (- ~ 0 ~ +)
            point_parameter.x = (-1) * _lrf_org_data[i] * cos((deg)*RGBD_D2R);
            point_parameter.y = _lrf_org_data[i] * sin((deg)*RGBD_D2R);
        }
        else{// Y axis (0 ~ +)
            point_parameter.x = _lrf_org_data[i] * cos((deg)*RGBD_D2R);
            point_parameter.y = (-1) * _lrf_org_data[i] * sin((deg)*RGBD_D2R);
        }

        mvec_point_vector.push_back(point_parameter);
    }

}

void CRGBD::EstimateLineEquation(){

}


//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CRGBD::run(){

    switch (fl_function_index){

    case THREAD_SEGNET_INDEX:
        SegnetFunction();
        break;

    default:
        break;
    }
}
