#include "CManipulation.h"

CManipulation::CManipulation(){

}

CManipulation::~CManipulation(){

    if(this->isRunning())
        this->terminate();
}

CManipulation::CManipulation(CLRF *_p_mani_lrf, CCamera *_p_camera, CKinova *_p_kinova, CVehicle *_p_vehicle, CVelodyne *_p_velodyne, CGripper* _p_gripper, CSSD* _ssd){

    mpc_lrf = _p_mani_lrf;
    mpc_camera = _p_camera;
    mpc_kinova = _p_kinova;
    mpc_vehicle = _p_vehicle;
    mpc_velodyne = _p_velodyne;
    mpc_gripper = _p_gripper;


    mpc_rgb_d = new CRGBD(_p_camera, _p_mani_lrf, _ssd);

    m_valve_size_result = 0;
    m_valve_size_graph_index = 0;

    connect(mpc_kinova, SIGNAL(SignalKinovaPosition(CartesianPosition)), this, SIGNAL(SignalKinovaPosition(CartesianPosition)));
    connect(mpc_rgb_d, SIGNAL(SignalLRFMapImage(cv::Mat)), this, SIGNAL(SignalLRFImage(cv::Mat)));
    connect(mpc_camera, SIGNAL(SignalCameraImage(cv::Mat)), this, SIGNAL(SignalCameraImage(cv::Mat)));

    connect(mpc_camera, SIGNAL(SignalCameraImage(cv::Mat)), this, SIGNAL(SignalCameraImage(cv::Mat)));
    connect(mpc_rgb_d, SIGNAL(SignalSegnetImage(cv::Mat)), this, SIGNAL(SignalSegnetImage(cv::Mat)));

    connect(mpc_gripper, SIGNAL(SignalEditeGripperStatus(GRIPPER_STATUS)), this, SIGNAL(SignalEditeGripperStatus(GRIPPER_STATUS)));
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

bool CManipulation::KinovaMoveUnitStep(double _x, double _y, double _z, double _th_x, double _th_y, double _th_z){

    if(!mpc_kinova->KinovaMoveUnitStep(_x, _y, _z, _th_x, _th_y, _th_z))
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

bool CManipulation::KinovaMoveUnitStepFw(){

    if(!mpc_kinova->KinovaMoveUnitStepFw())
        return false;

    return true;
}

bool CManipulation::KinovaMoveUnitStepBw(){

    if(!mpc_kinova->KinovaMoveUnitStepBw())
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
bool CManipulation::InitLRF(char* _dev_path, int _dev_type){

    if(!mpc_lrf->InitLRF(_dev_path, _dev_type))
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

bool CManipulation::GetLRFInfo(double &_slope, double &_distance, double _s_deg, double _e_deg){

    if(!InitLRF())
        return false;

    mpc_rgb_d->GetLRFInfo(_slope, _distance, _s_deg, _e_deg);

    return true;
}

bool CManipulation::GetHorizenDistance(double _inlier_distance,double& _horizen_distance, double& _s_inlier_deg, double& _e_inlier_deg,
                               double& _virt_s_deg, double& _virt_e_deg, double _s_deg, double _e_deg, int _sampling_loop){
    if(!InitLRF())
        return false;

    mpc_rgb_d->GetHorizenDistance(_inlier_distance, _horizen_distance, _s_inlier_deg, _e_inlier_deg,
                                  _virt_s_deg, _virt_e_deg, _s_deg, _e_deg, _sampling_loop);

    return true;
}

//Camera
bool CManipulation::InitCamera(){

    if(!mpc_camera->InitCamera("192.168.0.44"))
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

//End Effector

bool CManipulation::InitGripper(char* _device_port){

    if(!mpc_gripper->InitGripper(_device_port))
        return false;

    if(!mpc_gripper->IsGripperTorqueOn())
        mpc_gripper->GripperTorque(true);

    return true;
}

bool CManipulation::CloseGripper(){

    if(mpc_gripper->IsGripperInit())
        mpc_gripper->CloseGripper();

    return true;
}

bool CManipulation::GripperGoRelPose(double _deg){

    if(!mpc_gripper->GripperGoToThePositionLoadCheck_1(_deg, 200))
        return false;

    return true;
}

bool CManipulation::GripperGoThePose(int _pose_1, int _pose_2, int _load_thresh){

    if(!mpc_gripper->GripperGoToThePositionLoadCheck(_pose_1, _pose_2, _load_thresh))
        return false;

    return true;
}

bool CManipulation::GripperPresentPose(uint16_t& _pose){

    if(!mpc_gripper->IsDmxInit())
        return false;

    _pose = mpc_gripper->DynamixelPresentPosition();

    return true;
}
bool CManipulation::GripperPresentLoad(uint16_t& _load){

    if(!mpc_gripper->IsDmxInit())
        return false;

    _load = mpc_gripper->DynamixelPresentLoad();

    return true;
}

//----------------------------------------------------------------
// Option Function
//----------------------------------------------------------------
bool CManipulation::SelectMainFunction(int _fnc_index_){

    if(this->isRunning()){ //A thread is Running?
        return false;
    }

    if(_fnc_index_ == MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL;
        this->start();

        return true;
    }
    if(_fnc_index_ == MANIPUL_INX_LRF_KINOVA_HORIZEN_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_KINOVA_HORIZEN_CTRL;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_KINOVA_FORCE_CLRL){
        m_main_fnc_index = MANIPUL_INX_KINOVA_FORCE_CLRL;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_KINOVA_FORCE_CHECK){
        m_main_fnc_index = MANIPUL_INX_KINOVA_FORCE_CHECK;
        this->start();

        return true;
    }

    else if(_fnc_index_ == MANIPUL_INX_GRIPPER_FORCE_CLRL){
        m_main_fnc_index = MANIPUL_INX_GRIPPER_FORCE_CLRL;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_GRIPPER_MAGNET_CLRL){
        m_main_fnc_index = MANIPUL_INX_GRIPPER_MAGNET_CLRL;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_GRIPPER_VALVE_SIZE_RECOG){
        m_main_fnc_index = MANIPUL_INX_GRIPPER_VALVE_SIZE_RECOG;
        this->start();

        return true;
    }

    else if(_fnc_index_ == MANIPUL_INX_KINOVA_MANIPULATE){
        m_main_fnc_index = MANIPUL_INX_KINOVA_MANIPULATE;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_KINOVA_ROTATE_VALVE){
        m_main_fnc_index = MANIPUL_INX_KINOVA_ROTATE_VALVE;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_WRENCH_RECOGNITION){
        m_main_fnc_index = MANIPUL_INX_WRENCH_RECOGNITION;
        this->start();

        return true;
    }
    else
        return false;
}

void CManipulation::SetManipulationOption(LRF_KINOVA_VERTICAL_CTRL_STRUCT _lrf_kinova_option){

    mxt_lrf_kinova_vertical.lock();
    {
        mstruct_lrf_kinova_vertical = _lrf_kinova_option;
    }
    mxt_lrf_kinova_vertical.unlock();
}

LRF_KINOVA_VERTICAL_CTRL_STRUCT CManipulation::GetLRFKinovaVerticalOption(){

    LRF_KINOVA_VERTICAL_CTRL_STRUCT lrf_kinova_struct;

    mxt_lrf_kinova_vertical .lock();
    {
        lrf_kinova_struct = mstruct_lrf_kinova_vertical;
    }
    mxt_lrf_kinova_vertical.unlock();

    return lrf_kinova_struct;
}

void CManipulation::SetManipulationOption(LRF_KINOVA_HORIZEN_CTRL_STRUCT _lrf_kinova_option){

    mxt_lrf_kinova_horizen.lock();
    {
        mstruct_lrf_kinova_horizen = _lrf_kinova_option;
    }
    mxt_lrf_kinova_horizen.unlock();
}

LRF_KINOVA_HORIZEN_CTRL_STRUCT CManipulation::GetLRFKinovaHorizenOption(){

    LRF_KINOVA_HORIZEN_CTRL_STRUCT lrf_kinova_struct;

    mxt_lrf_kinova_horizen .lock();
    {
        lrf_kinova_struct = mstruct_lrf_kinova_horizen;
    }
    mxt_lrf_kinova_horizen.unlock();

    return lrf_kinova_struct;
}

void CManipulation::SetManipulationOption(KINOVA_FORCE_CTRL_STRUCT _kinova_force_ctrl){

    mxt_kinova_force_ctrl.lock();
    {
        mstruct_kinova_force_ctrl = _kinova_force_ctrl;
    }
    mxt_kinova_force_ctrl.unlock();
}

KINOVA_FORCE_CTRL_STRUCT CManipulation::GetKinovaForceCtrlOption(){

    KINOVA_FORCE_CTRL_STRUCT kinova_force_ctrl;

    mxt_kinova_force_ctrl .lock();
    {
        kinova_force_ctrl = mstruct_kinova_force_ctrl;
    }
    mxt_kinova_force_ctrl.unlock();

    return kinova_force_ctrl;
}

void CManipulation::SetManipulationOption(KINOVA_FORCE_CHECK_STRUCT _kinova_force_check){

    mxt_kinova_force_check.lock();
    {
        mstruct_kinova_force_check = _kinova_force_check;
    }
    mxt_kinova_force_check.unlock();
}

KINOVA_FORCE_CHECK_STRUCT CManipulation::GetKinovaForceCheckOption(){

    KINOVA_FORCE_CHECK_STRUCT kinova_force_check;

    mxt_kinova_force_check .lock();
    {
        kinova_force_check = mstruct_kinova_force_check;
    }
    mxt_kinova_force_check.unlock();

    return kinova_force_check;
}

void CManipulation::SetManipulationOption(GRIPPER_FORCE_CTRL_STRUCT _manipulation_option){

    mxt_gripper_force_ctrl.lock();
    {
        mstruct_gripper_force_ctrl = _manipulation_option;
    }
    mxt_gripper_force_ctrl.unlock();
}

GRIPPER_FORCE_CTRL_STRUCT CManipulation::GetGripperForceCtrlOption(){

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_ctrl;

    mxt_gripper_force_ctrl.lock();
    {
        gripper_force_ctrl = mstruct_gripper_force_ctrl;
    }
    mxt_gripper_force_ctrl.unlock();

    return gripper_force_ctrl;
}

void CManipulation::SetManipulationOption(GRIPPER_MAGNET_CTRL_STRUCT _manipulation_option){

    mxt_gripper_magnet_ctrl.lock();
    {
        mstruct_gripper_magnet_ctrl = _manipulation_option;
    }
    mxt_gripper_magnet_ctrl.unlock();
}

GRIPPER_MAGNET_CTRL_STRUCT CManipulation::GetGripperMagnetCtrlOption(){

    GRIPPER_MAGNET_CTRL_STRUCT gripper_magnet_ctrl;

    mxt_gripper_magnet_ctrl.lock();
    {
        gripper_magnet_ctrl = mstruct_gripper_magnet_ctrl;
    }
    mxt_gripper_magnet_ctrl.unlock();

    return gripper_magnet_ctrl;
}

void CManipulation::SetManipulationOption(KINOVA_DO_MANIPULATE_STRUCT _manipulation_option){

    mxt_kinova_manipulate.lock();
    {
        mstruct_kinova_manipulate = _manipulation_option;
    }
    mxt_kinova_manipulate.unlock();
}

KINOVA_DO_MANIPULATE_STRUCT CManipulation::GetKinovaManipulateOption(){

    KINOVA_DO_MANIPULATE_STRUCT kinova_manipulate;

    mxt_kinova_manipulate.lock();
    {
        kinova_manipulate = mstruct_kinova_manipulate;
    }
    mxt_kinova_manipulate.unlock();

    return kinova_manipulate;
}

void CManipulation::SetManipulationOption(KINOVA_ROTATE_VALVE_STRUCT _manipulation_option){

    mxt_kinova_rotate_valve.lock();
    {
        mstruct_kinova_rotate_valve= _manipulation_option;
    }
    mxt_kinova_rotate_valve.unlock();
}

KINOVA_ROTATE_VALVE_STRUCT CManipulation::GetKinovaRotateValveOption(){

    KINOVA_ROTATE_VALVE_STRUCT kinova_rotate_valve;

    mxt_kinova_rotate_valve.lock();
    {
        kinova_rotate_valve = mstruct_kinova_rotate_valve;
    }
    mxt_kinova_rotate_valve.unlock();

    return kinova_rotate_valve;
}

void CManipulation::SetManipulationOption(GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT _manipulation_option){

    mxt_gripper_kinova_valve_recog.lock();
    {
        mstruct_gripper_kinova_valve_recog = _manipulation_option;
    }
    mxt_gripper_kinova_valve_recog.unlock();
}

GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT CManipulation::GetGripperKinovaValveRecogOption(){

    GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT gripper_kinova_valve_recog;

    mxt_gripper_kinova_valve_recog.lock();
    {
        gripper_kinova_valve_recog = mstruct_gripper_kinova_valve_recog;
    }
    mxt_gripper_kinova_valve_recog.unlock();

    return gripper_kinova_valve_recog;
}

void CManipulation::SetManipulationOption(WRENCH_RECOGNITION _manipulation_option){

    mxt_wrench_recognition.lock();
    {
        mstruct_wrench_recognition = _manipulation_option;
    }
    mxt_wrench_recognition.unlock();
}

WRENCH_RECOGNITION CManipulation::GetWrenchRecognitionOption(){

    WRENCH_RECOGNITION wrench_recognition;

    mxt_wrench_recognition.lock();
    {
        wrench_recognition = mstruct_wrench_recognition;
    }
    mxt_wrench_recognition.unlock();

    return wrench_recognition;
}

//-------------------------------------------------
// Calculate Function
//-------------------------------------------------
void CManipulation::ValveSizeDataModelInit(QVector<double> _data, int _index/*mm*/){

    if(_index == 16){
        if(!mqvec_16mm.empty()){
            mqvec_16mm.clear();
            mqvec_sorted_16mm.clear();
        }

        mqvec_16mm = _data;
        mqvec_sorted_16mm = DataSort(mqvec_16mm);
    }

    if(_index == 17){
        if(!mqvec_17mm.empty()){
            mqvec_17mm.clear();
            mqvec_sorted_17mm.clear();
        }

        mqvec_17mm = _data;
        mqvec_sorted_17mm = DataSort(mqvec_17mm);
    }

    if(_index == 18){
        if(!mqvec_18mm.empty()){
            mqvec_18mm.clear();
            mqvec_sorted_18mm.clear();
        }

        mqvec_18mm = _data;
        mqvec_sorted_18mm = DataSort(mqvec_18mm);
    }

    if(_index == 19){
        if(!mqvec_19mm.empty()){
            mqvec_19mm.clear();
            mqvec_sorted_19mm.clear();
        }

        mqvec_19mm = _data;
        mqvec_sorted_19mm = DataSort(mqvec_19mm);
    }

    if(_index == 22){
        if(!mqvec_22mm.empty()){
            mqvec_22mm.clear();
            mqvec_sorted_22mm.clear();
        }

        mqvec_22mm = _data;
        mqvec_sorted_22mm = DataSort(mqvec_22mm);
    }

    if(_index == 24){
        if(!mqvec_24mm.empty()){
            mqvec_24mm.clear();
            mqvec_sorted_24mm.clear();
        }

        mqvec_24mm = _data;
        mqvec_sorted_24mm = DataSort(mqvec_24mm);
    }
}

QVector<double> CManipulation::DataSort(QVector<double> _data){

    for(int i = 0; i < _data.size(); i++){
        for(int j = i; j < _data.size(); j++){

            if(_data.at(i) > _data.at(j)){

                double tmp_data = 0;

                tmp_data = _data[i];
                _data[i] = _data[j];
                _data[j] = tmp_data;

            }
        }
    }

    return _data;
}

int CManipulation::DataAnalisys(QVector<double> _data){//For Valve Recognition, Result: 16 ~ 24(16mm, 17mm, 18mm, 19mm, 22mm, 24mm)

    QVector<double> x(_data.size());

    int valve_index = 0;
    int min_error = 999999;/*Just... Big Number*/
    int ary_error[6] = {-1, -1, -1, -1, -1, -1};
    int result_valve = 0;//16,17,18,19,22,24mm

    for(int i = 0; i < x.size(); i++){
        x[i] = i;
    }

    for(int i = 0; i < 35; i++){

        if(!mqvec_sorted_16mm.isEmpty()){
            ary_error[0] += fabs(_data[i] - mqvec_sorted_16mm[i]);
        }
        if(!mqvec_sorted_17mm.isEmpty()){
            ary_error[1] += fabs(_data[i] - mqvec_sorted_17mm[i]);
        }
        if(!mqvec_sorted_18mm.isEmpty()){
            ary_error[2] += fabs(_data[i] - mqvec_sorted_18mm[i]);
        }
        if(!mqvec_sorted_19mm.isEmpty()){
            ary_error[3] += fabs(_data[i] - mqvec_sorted_19mm[i]);
        }
        if(!mqvec_sorted_22mm.isEmpty()){
            ary_error[4] += fabs(_data[i] - mqvec_sorted_22mm[i]);
        }
        if(!mqvec_sorted_24mm.isEmpty()){
            ary_error[5] += fabs(_data[i] - mqvec_sorted_24mm[i]);
        }
    }

    for(int i = 0; i < 6/*Number of Valve*/; i++){
        if(ary_error[i] != -1){
            if(min_error > ary_error[i]){
                valve_index = i;
                min_error = ary_error[i];
            }
        }
    }

    if(min_error == 999999){
        return 0;
    }
    else
        result_valve = mary_valve_size[valve_index];

    return result_valve;
}

//----------------------------------------------------------------
// Main Function Result
//----------------------------------------------------------------
void CManipulation::SetMainFunctionResult(bool _result){

    fl_main_fnc_result = _result;
}

bool CManipulation::GetMainFunctionResult(){

    return fl_main_fnc_result;
}

int CManipulation::GetValveSizeRecogResult(){

    return m_valve_size_result;
}

void CManipulation::SetValveSizeRecogResult(int _result){
    m_valve_size_result = _result;
}

//----------------------------------------------------------------
// Main Function
//----------------------------------------------------------------
bool CManipulation::LRFKinovaVerticalControl(){

    LRF_KINOVA_VERTICAL_CTRL_STRUCT lrf_kinova_struct = GetLRFKinovaVerticalOption();

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!lrf_kinova_struct.sensor_option){
        if(!mpc_kinova->IsKinovaInitialized())
            return false;
    }


    do{
        double slope = 0;
        double current_distance = 0;

        mpc_rgb_d->GetLRFInfo(slope, current_distance, lrf_kinova_struct.s_deg, lrf_kinova_struct.e_deg, lrf_kinova_struct.inlier_lrf_dst);

        double current_error = lrf_kinova_struct.desired_distance - current_distance;/*mm*/

        lrf_kinova_struct.slope = slope;
        lrf_kinova_struct.current_distance = current_distance;

//        std::cout << "C-LRF Data : " << current_distance<<std::endl;
//        std::cout << "D-LRF Data : " << lrf_kinova_struct.desired_distance<<std::endl;

        if(fabs(current_error) < lrf_kinova_struct.error){
            break;
        }

        if(!lrf_kinova_struct.sensor_option){
            if(current_error < 0){
                mpc_kinova->KinovaMoveUnitStepFw();
            }
            else if(current_error > 0){
                mpc_kinova->KinovaMoveUnitStepBw();
            }

            if(lrf_kinova_struct.loop_sleep != 0)
                msleep(lrf_kinova_struct.loop_sleep/*msec*/);
        }

        emit SignalLRFKinovaVerticalStruct(lrf_kinova_struct);
    }
    while(true);

    return true;
}

bool CManipulation::LRFKinovaHorizenControl(){

    LRF_KINOVA_HORIZEN_CTRL_STRUCT lrf_kinova_struct = GetLRFKinovaHorizenOption();

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!lrf_kinova_struct.sensor_option){
        if(!mpc_kinova->IsKinovaInitialized())
            return false;
    }

    int count = 0;
    int sampling_loop = 1;

    double inlier_s_deg = 0;
    double inlier_e_deg = 0;
    double current_h_distance = 0;
    double inlier_deg_avr = 0;
    double inlier_deg_error = 0;

    double lnlier_deg_sum = 0;

    double s_virture_deg = 0;
    double e_virture_deg = 0;

    if(lrf_kinova_struct.desired_inlier_deg_avr == 0){

        switch(lrf_kinova_struct.wrench_hanger_index){

        case 1:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_1;
        case 2:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_2;
        case 3:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_3;
        case 4:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_4;
        case 5:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_5;
        case 6:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_6;
            break;
        }
    }

    do{
        inlier_s_deg = 0;
        inlier_e_deg = 0;
        current_h_distance = 0;
        inlier_deg_avr = 0;
        inlier_deg_error = 0;

        lnlier_deg_sum = 0;

        s_virture_deg = 0;
        e_virture_deg = 0;

        lrf_kinova_struct.inlier_deg_s_output = 0;
        lrf_kinova_struct.inlier_deg_e_output = 0;

        mpc_rgb_d->GetHorizenDistance(lrf_kinova_struct.inlier_lrf_dst, current_h_distance, inlier_s_deg, inlier_e_deg,
                                      s_virture_deg, e_virture_deg, lrf_kinova_struct.s_deg, lrf_kinova_struct.e_deg, sampling_loop);

        lrf_kinova_struct.inlier_deg_s_output = inlier_s_deg;
        lrf_kinova_struct.inlier_deg_e_output = inlier_e_deg;
        lrf_kinova_struct.current_h_distance = current_h_distance;

        lnlier_deg_sum = lrf_kinova_struct.inlier_deg_s_output + lrf_kinova_struct.inlier_deg_e_output;
        inlier_deg_avr = (double)(lnlier_deg_sum / 2.0);
        inlier_deg_error = inlier_deg_avr - lrf_kinova_struct.desired_inlier_deg_avr;

        emit SignalLRFKinovaHorizenStruct(lrf_kinova_struct);

        if(fabs(inlier_deg_error) < lrf_kinova_struct.error){
            if(count < 5){
                count++;
                sampling_loop = 5;
                usleep(500);
                continue;
            }
            else
                break;
        }
        else if(fabs(inlier_deg_error) < (lrf_kinova_struct.error*5)){
            sampling_loop = 5;
            usleep(500);
        }

        if(!lrf_kinova_struct.sensor_option){
            if(inlier_deg_error < 0){
                mpc_kinova->KinovaMoveUnitStepRi();
            }
            else if(inlier_deg_error > 0){
                mpc_kinova->KinovaMoveUnitStepLe();
            }

            if(lrf_kinova_struct.loop_sleep != 0)
                msleep(lrf_kinova_struct.loop_sleep/*msec*/);
        }

        count = 0;
    }
    while(true);

    return true;
}

bool CManipulation::KinovaForceCtrl(){

    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    int step_count = 0;
    double position_threshold = 0.03;

    KINOVA_FORCE_CTRL_STRUCT kinova_force_ctrl = GetKinovaForceCtrlOption();

    do{
        CartesianPosition current_pos = mpc_kinova->KinovaGetPosition();
        CartesianPosition cartesian_pos = mpc_kinova->KinovaGetCartesianForce();
        emit SignalKinovaForceVector(cartesian_pos);

        if(step_count > kinova_force_ctrl.step_count){
            return false;
        }

        if(kinova_force_ctrl.force_threshold_x != 0){

            if(kinova_force_ctrl.force_threshold_x > 0){
                if(cartesian_pos.Coordinates.X > kinova_force_ctrl.force_threshold_x){//Over Threshold X axis force
                    break;
                }
            }
            if(kinova_force_ctrl.force_threshold_x < 0){
                if(cartesian_pos.Coordinates.X < kinova_force_ctrl.force_threshold_x){//Over Threshold X axis force
                    break;
                }
            }

            if(position_threshold > fabs(kinova_force_ctrl.position_limit_x - current_pos.Coordinates.X))
                return false;
        }

        if(kinova_force_ctrl.force_threshold_y != 0){

            if(kinova_force_ctrl.force_threshold_y > 0){
                if(cartesian_pos.Coordinates.Y > kinova_force_ctrl.force_threshold_y){//Over Threshold X axis force
                    break;
                }
            }
            if(kinova_force_ctrl.force_threshold_y < 0){
                if(cartesian_pos.Coordinates.Y < kinova_force_ctrl.force_threshold_y){//Over Threshold X axis force
                    break;
                }
            }
            if(position_threshold > fabs(kinova_force_ctrl.position_limit_y - current_pos.Coordinates.Y))
                return false;
        }

        if(kinova_force_ctrl.force_threshold_z != 0){

            if(kinova_force_ctrl.force_threshold_z > 0){
                if(cartesian_pos.Coordinates.Z > kinova_force_ctrl.force_threshold_z){//Over Threshold X axis force
                    break;
                }
            }
            if(kinova_force_ctrl.force_threshold_z < 0){
                if(cartesian_pos.Coordinates.Z < kinova_force_ctrl.force_threshold_z){//Over Threshold X axis force
                    break;
                }
            }
            if(position_threshold > fabs(kinova_force_ctrl.position_limit_z - current_pos.Coordinates.Z))
                return false;
        }

        mpc_kinova->KinovaMoveUnitStep(kinova_force_ctrl.move_step_x, kinova_force_ctrl.move_step_y, kinova_force_ctrl.move_step_z);

        step_count++;
    }
    while(true);

    return true;
}

bool CManipulation::KinovaForceCheck(){

    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    int check_count = 0;

    KINOVA_FORCE_CHECK_STRUCT kinova_force_check = GetKinovaForceCheckOption();

    do{
        CartesianPosition cartesian_pos = mpc_kinova->KinovaGetCartesianForce();
        emit SignalKinovaForceVector(cartesian_pos);

        if(check_count > kinova_force_check.check_count)
            break;

        if(kinova_force_check.force_threshold_x != 0){
            if(fabs(cartesian_pos.Coordinates.X) > kinova_force_check.force_threshold_x){//Over Threshold X axis force
                return true;
            }
        }

        if(kinova_force_check.force_threshold_y != 0){
            if(fabs(cartesian_pos.Coordinates.Y) > kinova_force_check.force_threshold_y){//Over Threshold Y axis force
                return true;
            }
        }

        if(kinova_force_check.force_threshold_z != 0){
            if(fabs(cartesian_pos.Coordinates.Z) > kinova_force_check.force_threshold_z){//Over Threshold Z axis force
                return true;
            }
        }

        check_count++;
    }
    while(true);

    return false;
}

bool CManipulation::KinovaDoManipulate(){

    CartesianPosition kinova_pose;
    KINOVA_DO_MANIPULATE_STRUCT kinova_manipulate = GetKinovaManipulateOption();

    kinova_pose.Coordinates.X = kinova_manipulate.x;
    kinova_pose.Coordinates.Y = kinova_manipulate.y;
    kinova_pose.Coordinates.Z = kinova_manipulate.z;

    kinova_pose.Coordinates.ThetaZ = kinova_manipulate.roll;
    kinova_pose.Coordinates.ThetaY = kinova_manipulate.pitch;
    kinova_pose.Coordinates.ThetaX = kinova_manipulate.yaw;

//    mpc_kinova->KinovaDoManipulate(kinova_pose, kinova_manipulate.force_threshold);
    mpc_kinova->KinovaDoManipulate(kinova_pose, 2);

    return true;
}

bool CManipulation::KinovaRotateValveMotion(){

    KINOVA_ROTATE_VALVE_STRUCT kinova_rotate_valve = GetKinovaRotateValveOption();

    mpc_kinova->SetKinovaRotateValve(kinova_rotate_valve.using_current_coord, kinova_rotate_valve.init_angle, kinova_rotate_valve.center_x, kinova_rotate_valve.center_y, kinova_rotate_valve.center_z);

    if(kinova_rotate_valve.theta > 0)
        mpc_kinova->KinovaRotateValveMotion(VALVE_ROTATE_DIR(CW), kinova_rotate_valve.radius, kinova_rotate_valve.theta);

    else{
        kinova_rotate_valve.theta = (-1)* kinova_rotate_valve.theta;
        mpc_kinova->KinovaRotateValveMotion(VALVE_ROTATE_DIR(CCW), kinova_rotate_valve.radius, kinova_rotate_valve.theta);
    }

    return true;
}

bool CManipulation::GripperKinovaValveSizeRecognition(){

    if(!mpc_gripper->IsGripperInit())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT gripper_kinova_valve_recog = GetGripperKinovaValveRecogOption();

    double grasp_pose_1 = gripper_kinova_valve_recog.grasp_pose_1;
    double grasp_pose_2 = gripper_kinova_valve_recog.grasp_pose_2;

    double force_threshold = gripper_kinova_valve_recog.force_threshold;

    double release_pose_1 = gripper_kinova_valve_recog.release_pose_1;
    double release_pose_2 = gripper_kinova_valve_recog.release_pose_2;

//    int inlier_error = gripper_kinova_valve_recog.inlier_error;

//    double unit_rotation_angle = gripper_kinova_valve_recog.unit_rotation_angle;

    GRIPPER_STATUS gripper_status;

    QVector<double> gripper_data_x;
    QVector<double> gripper_data_y;

    double org_roll_pose = 0;
    CartesianPosition current_pose;
    current_pose = mpc_kinova->KinovaGetPosition();

    org_roll_pose = current_pose.Coordinates.ThetaZ;


    // -1: Add Graph
    emit SignalValveSizeData(gripper_data_x, gripper_data_y, -1);

    double diff_pose = 0;
    double diff_pose_1 = 0;
    double diff_pose_2 = 0;

    double roll_angle = 0;

    bool fl_gripper_1_not_reach = false;
    bool fl_gripper_2_not_reach = false;

    for(int i = 0; i < gripper_kinova_valve_recog.trial; i++){

        mpc_gripper->GripperGoToThePositionLoadCheck(grasp_pose_1, grasp_pose_2, force_threshold);

        gripper_status = mpc_gripper->GetGripperStatus();

        if(gripper_status.present_pose_1 < 1750){
            fl_gripper_1_not_reach = true;
        }

        if(gripper_status.present_pose_2 < 1750){
            fl_gripper_2_not_reach = true;
        }

        if(fl_gripper_1_not_reach || fl_gripper_2_not_reach ){

            double y_step = 0;
            double z_step = 0;

            if(fl_gripper_1_not_reach && !fl_gripper_2_not_reach){
                y_step = 2*VEL * cos(roll_angle);
                z_step = 2*VEL * sin(roll_angle);
            }
            else if(!fl_gripper_1_not_reach && fl_gripper_2_not_reach){
                y_step = (-1) * 2*VEL * cos(roll_angle);
                z_step = (-1) * 2*VEL * sin(roll_angle);
            }

            mpc_gripper->GripperGoToThePositionLoadCheck(release_pose_1, release_pose_2, force_threshold);

            mpc_kinova->KinovaMoveUnitStep(0, y_step, z_step);

            current_pose = mpc_kinova->KinovaGetPosition();
            mpc_kinova->KinovaDoManipulate(current_pose, 2);

            fl_gripper_1_not_reach = false;
            fl_gripper_2_not_reach = false;

            i -= 1;
            if(i < 0) i = 0;

            continue;
        }

        diff_pose_1 = fabs(gripper_status.present_pose_1 - grasp_pose_1);
        diff_pose_2 = fabs(gripper_status.present_pose_2 - grasp_pose_2);

        diff_pose = fabs(diff_pose_1 + diff_pose_2);

        gripper_data_x.push_back((double)i);
        gripper_data_y.push_back(diff_pose);

        emit SignalValveSizeData(gripper_data_x, gripper_data_y, m_valve_size_graph_index);

        mpc_gripper->GripperGoToThePositionLoadCheck(release_pose_1, release_pose_2, force_threshold);

        roll_angle += (KINOVA_PI / 36);
        current_pose.Coordinates.ThetaZ += (KINOVA_PI / 36);
        mpc_kinova->KinovaDoManipulate(current_pose, 3);

    };

    m_valve_size_graph_index++;

    SetManipulationOption(gripper_kinova_valve_recog);

    current_pose.Coordinates.ThetaZ = org_roll_pose;
    mpc_kinova->KinovaDoManipulate(current_pose, 3);

    int valve_size_recog = DataAnalisys(DataSort(gripper_data_y));

    SetValveSizeRecogResult(valve_size_recog);

    return true;
}

bool CManipulation::GripperForceCtrl(){

    if(!mpc_gripper->IsGripperInit())
        return false;
    if(!mpc_gripper->IsGripperTorqueOn())
        return false;

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_ctrl;

    gripper_force_ctrl = GetGripperForceCtrlOption();

    mpc_gripper->GripperGoToThePositionLoadCheck(gripper_force_ctrl.pose_1, gripper_force_ctrl.pose_2, gripper_force_ctrl.force_threshold);

    return true;
}

bool CManipulation::GripperMagnetCtrl(){

    if(!mpc_vehicle->IsConnected())
        return false;

    GRIPPER_MAGNET_CTRL_STRUCT gripper_magnet_ctrl;

    gripper_magnet_ctrl = GetGripperMagnetCtrlOption();

    mpc_vehicle->ActiveMagnet(gripper_magnet_ctrl.fl_magnet);

    return true;
}

bool CManipulation::WrenchRecognition(){

    if(!mpc_camera->isRunning())
        return false;

    WRENCH_RECOGNITION wrench_recognition = GetWrenchRecognitionOption();
//    wrench_recognition.valve_size;
//    mpc_rgb_d->SSD~~~

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CManipulation::run(){

    SetMainFunctionResult(false);

    switch (m_main_fnc_index){

    case MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL:
        LRFKinovaVerticalControl();
        break;
    case MANIPUL_INX_LRF_KINOVA_HORIZEN_CTRL:
        LRFKinovaHorizenControl();
        break;
    case MANIPUL_INX_KINOVA_FORCE_CLRL:
        SetMainFunctionResult(KinovaForceCtrl());
        break;
    case MANIPUL_INX_KINOVA_FORCE_CHECK:
        SetMainFunctionResult(KinovaForceCheck());
        break;
    case MANIPUL_INX_GRIPPER_FORCE_CLRL:
        GripperForceCtrl();
        break;
    case MANIPUL_INX_GRIPPER_MAGNET_CLRL:
        GripperMagnetCtrl();
        break;
    case MANIPUL_INX_GRIPPER_VALVE_SIZE_RECOG:
        GripperKinovaValveSizeRecognition();
        break;
    case MANIPUL_INX_KINOVA_MANIPULATE:
        KinovaDoManipulate();
        break;
    case MANIPUL_INX_KINOVA_ROTATE_VALVE:
        KinovaRotateValveMotion();
        break;
    case MANIPUL_INX_WRENCH_RECOGNITION:
        WrenchRecognition();
        break;
    default:
        break;

    }
}

