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

    int s_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_index -s_index + 1;

    long* lrf_distance = new long[number_of_point];

    LINE_PARAM optimal_line_eq;

    if(!mpc_lrf->GetLRFData(mary_lrf_distance)){
        std::cout << "LRF : GetLRFData Error" << std::endl;
        return;
    }
    else{
        std::vector<POINT_PARAM> point_vec;

        memcpy(lrf_distance, &mary_lrf_distance[s_index], sizeof(long)*(number_of_point));

        ClaculateLRFHeightDistance(lrf_distance, _s_deg, _e_deg, point_vec);

        optimal_line_eq = EstimateLineEquation(point_vec);
    }

    delete[] lrf_distance;

    _slope = optimal_line_eq.yaw;
    _distance = optimal_line_eq.Distance;

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

void CRGBD::ClaculateLRFHeightDistance(long* _lrf_org_data, double _s_deg, double _e_deg, std::vector<POINT_PARAM>& _point_vec){

    int number_of_point = int((_e_deg -_s_deg) / ANGLE_RESOLUTION) + 1;

    for(int i = 0; i < number_of_point; i++){
        POINT_PARAM point_parameter;

        double deg = i*ANGLE_RESOLUTION + _s_deg;

        point_parameter.x = _lrf_org_data[i] * cos((deg)*RGBD_D2R);
        point_parameter.y = _lrf_org_data[i] * sin((deg)*RGBD_D2R);

        _point_vec.push_back(point_parameter);
    }
}

LINE_PARAM CRGBD::EstimateLineEquation(std::vector<POINT_PARAM>& _point_vec){

    int iteration = 1000;

    int random_num_1 = 0;
    int random_num_2 = 0;

    int random_max_num = _point_vec.size();

    LINE_PARAM optimal_line_eq;
    LINE_PARAM line_parameter;

    std::vector<LINE_PARAM> line_eq_vector;

    std::srand((unsigned int)time(NULL));

    //Get Line Equations candidate using random sampling
    for(int i = 0; i < iteration; i++){

        random_num_1 = std::rand() % random_max_num;
        random_num_2 = std::rand() % random_max_num;

        double line_son = (_point_vec.at(random_num_1).y - _point_vec.at(random_num_2).y);
        double line_parent = (_point_vec.at(random_num_1).x - _point_vec.at(random_num_2).x);

        line_parameter.a = line_son / line_parent;

        line_parameter.b = _point_vec.at(random_num_2).y - (_point_vec.at(random_num_2).x * line_parameter.a);

        if(line_parameter.b < 0)
            std::cout << "What!!?" << std::endl;

        line_parameter.num_inlier = 0;

        line_eq_vector.push_back(line_parameter);
    }

    double line_parm_a = 0.0;
    double line_parm_b = 0.0;

    double line_distance = 0;

    double line_max_count = 0;

    double line_inlier_standard = 10;

    for(unsigned int i = 0; i < line_eq_vector.size(); i++){

        line_parm_a = line_eq_vector.at(i).a;
        line_parm_b = line_eq_vector.at(i).b;

        for(unsigned int j = 0; j < _point_vec.size(); j++){

            double line_son = 0;
            double line_parent = 0;

            line_son = fabs(line_parm_a*_point_vec.at(j).x - _point_vec.at(j).y + line_parm_b);

            line_parent = sqrt(pow(line_parm_a,2.0) + pow(1.0,2.0));

            line_distance = (line_son / line_parent);

            if(line_distance <= line_inlier_standard){
                line_eq_vector.at(i).num_inlier++;
            }
        }
    }

    for(unsigned int i = 0; i < line_eq_vector.size(); i++){
        if(line_max_count < line_eq_vector.at(i).num_inlier){
            line_max_count = line_eq_vector.at(i).num_inlier;
            optimal_line_eq = line_eq_vector.at(i);
        }
    }

    double yaw_rad_line = atan(optimal_line_eq.a);

    optimal_line_eq.yaw = (yaw_rad_line * 180) / 3.14159265359;

    optimal_line_eq.Distance = optimal_line_eq.b;


    return optimal_line_eq;
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
