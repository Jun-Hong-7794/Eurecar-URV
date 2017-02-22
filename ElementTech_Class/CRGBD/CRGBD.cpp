#include "CRGBD.h"

CRGBD::CRGBD(){
    fl_function_index = 0;
}

CRGBD::CRGBD(CCamera* _pc_camera, CLRF* _pc_lrf, CSSD* _ssd){

    mpc_lrf = _pc_lrf;
    mpc_camera = _pc_camera;
    mpc_ssd = _ssd;
}

CRGBD::CRGBD(CLRF* _pc_lrf){
    mpc_lrf = _pc_lrf;
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
//    cv::Mat segnet_image;
    vector<vector<int>> bb_info;
    while(fl_function_index == THREAD_SEGNET_INDEX){
        if(!mpc_camera->GetCameraImage(camera_image))
            return;
//        segnet_image = mc_segnet.GetSegnetImage(camera_image);

        bb_info = mpc_ssd->GetSSDImage(camera_image);

        emit SignalSegnetImage(camera_image);
        msleep(30);
    }

    return;
}

//-------------------------------------------------
// Element Tech Function
//-------------------------------------------------

void CRGBD::GetLRFInfo(double &_slope, double &_distance, double _s_deg, double _e_deg, int _inlier_lrf_dst){

    if((_s_deg < 0 || _e_deg > 180) || (_s_deg >= _e_deg)){//Out Of Range . . .

        _slope = 0;
        _distance = 0;

        std::cout << "LRF Warning: Out of Range" << std::endl;
        return;
    }

    //Convert to Original Coordinate

    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_lrf_index -s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    int line_eq_num_of_samples = 2;
    std::vector<LINE_PARAM> optimal_line_eq_vec;

    for(int i = 0; i < line_eq_num_of_samples; i++){
        if(!mpc_lrf->GetLRFData(mary_lrf_distance)){
            std::cout << "LRF : GetLRFData Error" << std::endl;
            return;
        }
        else{
            int s_index = -1;
            int inlier_v_distance = (int)(_inlier_lrf_dst / sin(_s_deg * RGBD_D2R)) + 200/*mm*/;
            std::vector<POINT_PARAM> point_vec;

            memcpy(lrf_distance, &mary_lrf_distance[s_lrf_index], sizeof(long)*(number_of_point));

            ClaculateLRFHeightDistance(lrf_distance, _s_deg, _e_deg, s_index, point_vec, inlier_v_distance);

            optimal_line_eq_vec.push_back(EstimateLineEquation(point_vec));

            emit SignalLRFMapImage(LRFDataToMat(point_vec, _inlier_lrf_dst, 1000));
        }
    }

    delete[] lrf_distance;

    for(int i = 0; i < line_eq_num_of_samples; i++){
        _slope += optimal_line_eq_vec.at(i).yaw;
        _distance += optimal_line_eq_vec.at(i).Distance;
    }

    _slope /= line_eq_num_of_samples;
    _distance /= line_eq_num_of_samples;

    return;
}

void CRGBD::GetHorizenDistance(double _inlier_distance,double& _horizen_distance, double& _s_inlier_deg, double& _e_inlier_deg,
                               double& _virt_s_deg, double& _virt_e_deg, double _s_deg, double _e_deg, int _sampling_loop){

    //Convert to Original Coordinate
    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_lrf_index - s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    std::vector<POINT_PARAM> point_vec;
    if(!mpc_lrf->GetLRFData(mary_lrf_distance)){
        std::cout << "LRF : GetLRFData Error" << std::endl;
        return;
    }
    else{
        int s_inlier_deg = 0;
        int e_inlier_deg = 0;
        for(int i = 0; i < _sampling_loop; i++){
            int s_index = -1;
            int s_inlier_inx = 0;
            int e_inlier_inx = 0;
            int inlier_v_distance = (int)(_inlier_distance / sin(_s_deg * RGBD_D2R)) + 200/*mm*/;

            memcpy(lrf_distance, &mary_lrf_distance[s_lrf_index], sizeof(long)*(number_of_point));

            ClaculateLRFHeightDistance(lrf_distance, _s_deg, _e_deg, s_index, point_vec, inlier_v_distance);

            if(point_vec.size() == 0){
                _s_inlier_deg = 0;
                _e_inlier_deg = 0;
                std::cout << "Error: ClaculateLRFHeightDistance" << std::endl;
                return;
            }
            ClaculateHorizenDistance(point_vec, _inlier_distance, _horizen_distance, s_inlier_inx, e_inlier_inx);

            ClaculateVirtualHorizenInfo(_virt_s_deg, _virt_e_deg, point_vec.at(s_inlier_inx).x, point_vec.at(e_inlier_inx).x);

            emit SignalLRFMapImage(LRFDataToMat(point_vec, _inlier_distance, 1000));

            s_inlier_deg += _s_deg + (s_inlier_inx + s_index) * (ANGLE_RESOLUTION);
            e_inlier_deg += _s_deg + (e_inlier_inx + s_index) * (ANGLE_RESOLUTION); // s_deg is right
        }

        if(_sampling_loop != 0){
            s_inlier_deg /= _sampling_loop;
            e_inlier_deg /= _sampling_loop;

            _s_inlier_deg = s_inlier_deg;
            _e_inlier_deg = e_inlier_deg;
        }
    }

    return;
}
void CRGBD::LocalizationOnPanel(LOCALIZATION_INFO_ON_PANEL &_info,int _mode,
                                double _s_deg/*deg*/, double _e_deg/*deg*/, int _inlier_dst/*mm*/,
                                int _current_v_dst/*mm, for const mode*/, double _current_ang/*deg for const mode*/){

    //Convert to Original Coordinate
    int s_index = -1;
    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int s_inlier_inx = 0;
    int e_inlier_inx = 0;

    double s_inlier_deg = 0;
    double e_inlier_deg = 0;

    double horizen_distance = 0;

    int current_v_dst = 0;
    double current_ang = 0;

    int number_of_point = e_lrf_index - s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    std::vector<POINT_PARAM> point_vec;

    std::vector<LINE_PARAM> optimal_line_eq_vec;

    int sample_loop = 3;

    int v_distance = 0;
    double angle = 0;

    if(!mpc_lrf->GetLRFData(mary_lrf_distance)){
        std::cout << "LRF : GetLRFData Error" << std::endl;
        delete[] lrf_distance;
        return;
    }
    else{
        memcpy(lrf_distance, &mary_lrf_distance[s_lrf_index], sizeof(long)*(number_of_point));

        if((_mode & 0x0f00) == L_M_VALUE_RANSAC){
                LINE_PARAM optimal_line_eq;
                mpc_lrf->GetLRFData(mary_lrf_distance);

                ClaculateLRFHeightDistance(lrf_distance, _s_deg, _e_deg, s_index, point_vec, _inlier_dst);

                optimal_line_eq = EstimateLineEquation(point_vec);

            current_v_dst = optimal_line_eq.Distance + 10/*offset*/;
            current_ang = optimal_line_eq.yaw;
        }
        else if((_mode & 0x0f00) == L_M_VALUE_CONSTANT){
            current_v_dst = _current_v_dst;
            current_ang = _current_ang;
        }

        _info.vertical_dst = current_v_dst;
        _info.angle = current_ang;

        if((_mode & 0xf000) == L_M_ROUGH){// Only RANSAC Mode, s_deg: 10, e_deg: 170
            std::vector<POINT_PARAM>::iterator iter_end = point_vec.end();
            _info.horizen__dst =  (*iter_end).x;
        }
        else if((_mode & 0xf000) == L_M_PRECISE){

            ClaculateHorizenDistance(point_vec, 800/*mm*/, horizen_distance, s_inlier_inx, e_inlier_inx);

            s_inlier_deg = _s_deg + (s_inlier_inx + s_index) * (ANGLE_RESOLUTION);
            e_inlier_deg = _s_deg + (e_inlier_inx + s_index) * (ANGLE_RESOLUTION); // s_deg is right

            if((_mode & 0x000f) == L_M_DIR_LEFT){// s_deg: 90, e_deg: 170, Using e_inlier_inx
                //Assume, always e_inlier_deg > 90
                e_inlier_deg = (e_inlier_deg - 90);
                _info.horizen__dst = fabs( tan((RGBD_D2R* e_inlier_deg))* current_v_dst );

            }
            else if((_mode & 0x000f) == L_M_DIR_RIGHT){// s_deg: 10, e_deg: 90, Using s_inlier_inx
                //Assume, always s_inlier_deg < 90
                s_inlier_deg = (90 - s_inlier_deg);
                _info.horizen__dst = 1000/*mm, panel size*/ - fabs( tan((RGBD_D2R* s_inlier_deg))* current_v_dst );
            }
        }
    }

    delete[] lrf_distance;
}

cv::Mat CRGBD::GetSegnetImage(cv::Mat _org_img){

    cv::Mat segnet_image;
    segnet_image = mc_segnet.GetSegnetImage(_org_img);

    return segnet_image;
}

//-------------------------------------------------
// Calculation Function
//-------------------------------------------------

void CRGBD::ClaculateLRFHeightDistance(long* _lrf_org_data, double _s_deg, double _e_deg, int& _s_index, std::vector<POINT_PARAM>& _point_vec, int _inlier_lrf_dst){

    int number_of_point = int((_e_deg -_s_deg) / ANGLE_RESOLUTION) + 1;

    for(int i = 0; i < number_of_point; i++){
        POINT_PARAM point_parameter;

        double deg = i*ANGLE_RESOLUTION + _s_deg;

        if(_lrf_org_data[i] > _inlier_lrf_dst)
            continue;

        if(_s_index < 0)
            _s_index = i;

        point_parameter.x = _lrf_org_data[i] * cos((deg)*RGBD_D2R);
        point_parameter.y = _lrf_org_data[i] * sin((deg)*RGBD_D2R);

        _point_vec.push_back(point_parameter);
    }
}

LINE_PARAM CRGBD::EstimateLineEquation(std::vector<POINT_PARAM>& _point_vec){

    int iteration = 500;

    int random_num_1 = 0;
    int random_num_2 = 0;

    int random_max_num = _point_vec.size();

    LINE_PARAM optimal_line_eq;
    LINE_PARAM line_parameter;

    if(random_max_num <= 0){
        optimal_line_eq.a = 999;
        optimal_line_eq.b = -999;
        optimal_line_eq.yaw = 999;
        return optimal_line_eq;
    }
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

//        if(line_parameter.b < 0)
//            std::cout << "What!!?" << std::endl;

        line_parameter.num_inlier = 0;

        line_eq_vector.push_back(line_parameter);
    }

    double line_parm_a = 0.0;
    double line_parm_b = 0.0;

    double line_distance = 0;

    double line_max_count = 0;

    double line_inlier_standard = 4;

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

void CRGBD::ClaculateHorizenDistance(std::vector<POINT_PARAM>& _point_vec, double _inlier_distance,double& _horizen_distance, int& _s_inlier_inx, int& _e_inlier_inx){

    unsigned int s_inlier_index = 0;
    unsigned int e_inlier_index = 0;
    double max_distance = 0;
    double min_distance = 9999;
    for(unsigned int i = 0; i < _point_vec.size(); i++){

        if(_inlier_distance > _point_vec.at(i).y){

            int c_pow2 = pow(_point_vec.at(i).x, 2) + pow(_point_vec.at(i).y, 2);

            if(c_pow2 > pow(1500/*mm*/, 2))
                continue;

            if(_point_vec.at(i).x > max_distance){
                max_distance = _point_vec.at(i).x;
                s_inlier_index = i;
            }
            if(_point_vec.at(i).x < min_distance){
                min_distance = _point_vec.at(i).x;
                e_inlier_index = i;
            }
        }
    }

    if(s_inlier_index > e_inlier_index){
        _horizen_distance = 0;

        _s_inlier_inx = 0;
        _e_inlier_inx = 0;

        return;
    }

    _horizen_distance = fabs(max_distance) + fabs(min_distance);

    _s_inlier_inx = s_inlier_index;
    _e_inlier_inx = e_inlier_index;
}

cv::Mat CRGBD::LRFDataToMat(std::vector<POINT_PARAM> _point_vec, double _inlier_distance, double _max_distance){

    cv::Mat lrf_map = cv::Mat::zeros(480,640,CV_8UC3);
    //Using [0~319(+)]{y},  [0~319(-), 320~639(+)]{x}

    for(unsigned int i = 0; i < _point_vec.size(); i++){

        if((fabs(_point_vec.at(i).x) > _max_distance))//Out of range
            continue;
        if(fabs(_point_vec.at(i).y) > _max_distance || (fabs(_point_vec.at(i).y) > _inlier_distance))//Out of range
            continue;

        int image_x = 320 + (int)((_point_vec.at(i).x * 320) / _max_distance);
        int image_y = 320 - (int)((_point_vec.at(i).y * 320) / _max_distance);

        cv::Point2d lrf_point;

        lrf_point.x = image_x;
        lrf_point.y = image_y;

        cv::circle(lrf_map, lrf_point, 2, cv::Scalar(0, 0, 255));
    }

    return lrf_map;
}

void CRGBD::ClaculateVirtualHorizenInfo(double& _vir_s_deg, double& _vir_e_deg,
                                 double _s_x_distance, double _e_x_distance, double _vir_v_distance){


    double s_tan = _s_x_distance / _vir_v_distance;
    double s_rad = 0;

    double e_tan = _e_x_distance / _vir_v_distance;
    double e_rad = 0;

    s_tan = s_tan > 0 ? s_tan : (-1) * s_tan;
    e_tan = e_tan > 0 ? e_tan : (-1) * e_tan;

    s_rad = atan(s_tan);
    e_rad = atan(e_tan);

    _vir_s_deg = _s_x_distance > 0 ? (s_rad * RGBD_R2D) : (180 - (s_rad * RGBD_R2D));
    _vir_e_deg = _e_x_distance > 0 ? (e_rad * RGBD_R2D) : (180 - (e_rad * RGBD_R2D));
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
