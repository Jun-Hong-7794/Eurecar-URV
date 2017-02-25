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
    mpc_ssd = _ssd;

    mpc_rgb_d = new CRGBD(_p_camera, _p_mani_lrf, _ssd);

    m_valve_size_result = 0;
    m_valve_size_graph_index = 0;

    m_valve_size = 19;

    m_mat_panel_model = cv::Mat::zeros(640,1280,CV_8UC3);

    connect(mpc_kinova, SIGNAL(SignalKinovaPosition(CartesianPosition)), this, SIGNAL(SignalKinovaPosition(CartesianPosition)));
    connect(mpc_rgb_d, SIGNAL(SignalLRFMapImage(cv::Mat)), this, SIGNAL(SignalLRFImage(cv::Mat)));
    connect(mpc_camera, SIGNAL(SignalCameraImage(cv::Mat)), this, SIGNAL(SignalCameraImage(cv::Mat)));

    connect(mpc_camera, SIGNAL(SignalCameraImage(cv::Mat)), this, SIGNAL(SignalCameraImage(cv::Mat)));
    connect(mpc_rgb_d, SIGNAL(SignalSegnetImage(cv::Mat)), this, SIGNAL(SignalSegnetImage(cv::Mat)));

    connect(mpc_gripper, SIGNAL(SignalEditeGripperStatus(GRIPPER_STATUS)), this, SIGNAL(SignalEditeGripperStatus(GRIPPER_STATUS)));

    connect(mpc_kinova, SIGNAL(SignalKinovaForce(CartesianPosition)), this, SIGNAL(SignalEditeGripperStatus(GRIPPER_STATUS)));

    MakePanelModel(19,3);
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

    msleep(500);

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

bool CManipulation::KinovaRotateBase(double _rot_deg){

    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    mpc_kinova->KinovaRotateBase(_rot_deg);

    return true;
}

bool CManipulation::KinovaForceCheck(double _force_x, double _force_y, double _force_z){ // true: force occured!

    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    CartesianPosition cartesian_pos = mpc_kinova->KinovaGetCartesianForce();

    msleep(30);

    if(_force_x != 0){
        if(fabs(cartesian_pos.Coordinates.X) > _force_x){//Over Threshold X axis force
            return true;
        }
    }

    if(_force_y != 0){
        if(fabs(cartesian_pos.Coordinates.Y) > _force_y){//Over Threshold Y axis force
            return true;
        }
    }

    if(_force_z != 0){
        if(fabs(cartesian_pos.Coordinates.Z) > _force_z){//Over Threshold Z axis force
                return true;
        }
    }

    return false;
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

bool CManipulation::LocalizationOnPanel(int _mode, double _s_deg/*deg*/,
                         double _e_deg/*deg*/, int _inlier_dst/*mm*/, int _current_v_dst/*mm, for const mode*/, double _current_ang/*deg for const mode*/){

    LOCALIZATION_INFO_ON_PANEL info;

    mpc_rgb_d->LocalizationOnPanel(info, _mode, _s_deg, _e_deg, _inlier_dst, _current_v_dst,_current_ang);

    PanelModeling(19, info.vertical_dst, info.horizen__dst, info.angle);

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

bool CManipulation::InitRotator(char* _device_port){

    if(!mpc_gripper->InitRotator(_device_port))
        return false;

    return true;
}

bool CManipulation::CloseRotator(){

    mpc_gripper->CloseRotator();

    return true;
}

bool CManipulation::RotatorGoThePose(int _step){

    if(!mpc_gripper->RotatorGoToThePosition(_step))
        return false;

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

    if(_fnc_index_ == MANIPUL_INX_LRF_K_ANGLE_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_K_ANGLE_CTRL;
        this->start();

        return true;
    }

    if(_fnc_index_ == MANIPUL_INX_LRF_K_VEHICLE_HORIZEN_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_K_VEHICLE_HORIZEN_CTRL;
        this->start();

        return true;
    }
    if(_fnc_index_ == MANIPUL_INX_LRF_K_VEHICLE_ANGLE_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_K_VEHICLE_ANGLE_CTRL;
        this->start();

        return true;
    }
    if(_fnc_index_ == MANIPUL_INX_LRF_KINOVA_ANGLE_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_KINOVA_ANGLE_CTRL;
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
    else if(_fnc_index_ == MANIPUL_INX_KINOVA_FIT_TO_VALVE){
        m_main_fnc_index = MANIPUL_INX_KINOVA_FIT_TO_VALVE;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_WRENCH_RECOGNITION){
        m_main_fnc_index = MANIPUL_INX_WRENCH_RECOGNITION;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_LRF_KINOVA_WRENCH_LOCATION){
        m_main_fnc_index = MANIPUL_INX_LRF_KINOVA_WRENCH_LOCATION;
        this->start();

        return true;
    }

    else if(_fnc_index_ == MANIPUL_INX_KINOVA_ALIGN_TO_PANEL){
        m_main_fnc_index = MANIPUL_INX_KINOVA_ALIGN_TO_PANEL;
        this->start();

        return true;
    }

    else if(_fnc_index_ == MANIPUL_INX_LRF_K_VERTIVAL_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_K_VERTIVAL_CTRL;
        this->start();

        return true;
    }

    else if(_fnc_index_ == MANIPUL_INX_LRF_K_HORIZEN_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_K_HORIZEN_CTRL;
        this->start();

        return true;
    }

    else if(_fnc_index_ == MANIPUL_INX_LRF_V_ANGLE_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_V_ANGLE_CTRL;
        this->start();

        return true;
    }

    else if(_fnc_index_ == MANIPUL_INX_LRF_V_HORIZEN_CTRL){
        m_main_fnc_index = MANIPUL_INX_LRF_V_HORIZEN_CTRL;
        this->start();

        return true;
    }

    else if(_fnc_index_ == SENSING_LRF_PANEL_LOCALIZATION){
        m_main_fnc_index = SENSING_LRF_PANEL_LOCALIZATION;
        this->start();

        return true;
    }

    else
        return false;
}

void CManipulation::SetManipulationOption(LRF_SENSING_INFO_STRUCT _manipulation_option){

    mxt_lrf_sensing_info.lock();
    {
        mstruct_lrf_sensing_info = _manipulation_option;
    }
    mxt_lrf_sensing_info.unlock();
}

LRF_SENSING_INFO_STRUCT CManipulation::GetLRFSensingInfo(){

    LRF_SENSING_INFO_STRUCT lrf_sensing_info_struct;

    mxt_lrf_sensing_info.lock();
    {
        lrf_sensing_info_struct = mstruct_lrf_sensing_info;
    }
    mxt_lrf_sensing_info.unlock();

    return lrf_sensing_info_struct;
}

/*LRF Ctrl New Version*/
//---------------Kinova---------------//
void CManipulation::SetManipulationOption(LRF_K_V_CTRL_STRUCT _manipulation_option){

    mxt_lrf_k_v_ctrl .lock();
    {
        mstruct_lrf_k_v_ctrl = _manipulation_option;
    }
    mxt_lrf_k_v_ctrl.unlock();
}

LRF_K_V_CTRL_STRUCT CManipulation::GetLRFKVerticalCtrlOption(){

    LRF_K_V_CTRL_STRUCT lrf_kinova_struct;

    mxt_lrf_k_v_ctrl .lock();
    {
        lrf_kinova_struct = mstruct_lrf_k_v_ctrl;
    }
    mxt_lrf_k_v_ctrl.unlock();

    return lrf_kinova_struct;
}

void CManipulation::SetManipulationOption(LRF_K_H_CTRL_STRUCT _manipulation_option){

    mxt_lrf_k_h_ctrl .lock();
    {
        mstruct_lrf_k_h_ctrl = _manipulation_option;
    }
    mxt_lrf_k_h_ctrl.unlock();
}

LRF_K_H_CTRL_STRUCT CManipulation::GetLRFKHorizenCtrlOption(){

    LRF_K_H_CTRL_STRUCT lrf_kinova_struct;

    mxt_lrf_k_h_ctrl .lock();
    {
       lrf_kinova_struct = mstruct_lrf_k_h_ctrl;
    }
    mxt_lrf_k_h_ctrl.unlock();

    return lrf_kinova_struct;
}

void CManipulation::SetManipulationOption(LRF_K_A_CTRL_STRUCT _manipulation_option){

    mxt_lrf_k_a_ctrl .lock();
    {
       mstruct_lrf_k_a_ctrl = _manipulation_option;
    }
    mxt_lrf_k_a_ctrl.unlock();
}

LRF_K_A_CTRL_STRUCT CManipulation::GetLRFKAngleCtrlOption(){

    LRF_K_A_CTRL_STRUCT lrf_kinova_struct;

    mxt_lrf_k_a_ctrl .lock();
    {
       lrf_kinova_struct = mstruct_lrf_k_a_ctrl;
    }
    mxt_lrf_k_a_ctrl.unlock();

    return lrf_kinova_struct;
}

//---------------Vehicle---------------//
void CManipulation::SetManipulationOption(LRF_V_A_CTRL_STRUCT _manipulation_option){

    mxt_lrf_v_a_ctrl .lock();
    {
       mstruct_lrf_v_a_ctrl = _manipulation_option;
    }
    mxt_lrf_v_a_ctrl.unlock();
}

LRF_V_A_CTRL_STRUCT CManipulation::GetLRFVAngleCtrlOption(){

    LRF_V_A_CTRL_STRUCT lrf_kinova_struct;

    mxt_lrf_v_a_ctrl .lock();
    {
       lrf_kinova_struct = mstruct_lrf_v_a_ctrl;
    }
    mxt_lrf_v_a_ctrl.unlock();

    return lrf_kinova_struct;
}

void CManipulation::SetManipulationOption(LRF_V_H_CTRL_STRUCT _manipulation_option){

    mxt_lrf_v_h_ctrl .lock();
    {
        mstruct_lrf_v_h_ctrl = _manipulation_option;
    }
    mxt_lrf_v_h_ctrl.unlock();
}

LRF_V_H_CTRL_STRUCT CManipulation::GetLRFVHorizenCtrlOption(){

    LRF_V_H_CTRL_STRUCT lrf_kinova_struct;

    mxt_lrf_v_h_ctrl .lock();
    {
       lrf_kinova_struct = mstruct_lrf_v_h_ctrl;
    }
    mxt_lrf_v_h_ctrl.unlock();

    return lrf_kinova_struct;
}

//////////////////////////////////////////////////////////


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

void CManipulation::SetManipulationOption(LRF_K_VEHICLE_ANGLE_STRUCT _manipulation_option){

    mxt_lrf_k_vehicle_angle .lock();
    {
        mstruct_lrf_k_vehicle_angle = _manipulation_option;
    }
    mxt_lrf_k_vehicle_angle.unlock();
}

LRF_K_VEHICLE_ANGLE_STRUCT CManipulation::GetLRFVehicleAngleOption(){

    LRF_K_VEHICLE_ANGLE_STRUCT lrf_kinova_struct;

    mxt_lrf_k_vehicle_angle .lock();
    {
        lrf_kinova_struct = mstruct_lrf_k_vehicle_angle;
    }
    mxt_lrf_k_vehicle_angle.unlock();

    return lrf_kinova_struct;
}

void CManipulation::SetManipulationOption(LRF_K_VEHICLE_HORIZEN_STRUCT _manipulation_option){

    mxt_lrf_k_vehicle_horizen.lock();
    {
        mstruct_lrf_k_vehicle_horizen = _manipulation_option;
    }
    mxt_lrf_k_vehicle_horizen.unlock();

}

LRF_K_VEHICLE_HORIZEN_STRUCT CManipulation::GetLRFVehicleHorizenOption(){

    LRF_K_VEHICLE_HORIZEN_STRUCT lrf_kinova_struct;

    mxt_lrf_k_vehicle_horizen.lock();
    {
        lrf_kinova_struct = mstruct_lrf_k_vehicle_horizen;
    }
    mxt_lrf_k_vehicle_horizen.unlock();

    return lrf_kinova_struct;
}


void CManipulation::SetManipulationOption(LRF_KINOVA_WRENCH_LOCATION_STRUCT _manipulation_option){

    mxt_lrf_kinova_wrench.lock();
    {
        mstruct_lrf_kinova_wrench = _manipulation_option;
    }
    mxt_lrf_kinova_wrench.unlock();
}

LRF_KINOVA_WRENCH_LOCATION_STRUCT CManipulation::GetLRFKinovaWrenchLocationOption(){

    LRF_KINOVA_WRENCH_LOCATION_STRUCT lrf_kinova_struct;

    mxt_lrf_kinova_wrench.lock();
    {
        lrf_kinova_struct = mstruct_lrf_kinova_wrench;
    }
    mxt_lrf_kinova_wrench.unlock();

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

void CManipulation::SetManipulationOption(KINOVA_FIT_TO_VALVE_POSE_STRUCT _manipulation_option){

    mxt_fit_to_valve_pose.lock();
    {
        mstruct_fit_to_valve_pose = _manipulation_option;
    }
    mxt_fit_to_valve_pose.unlock();
}

KINOVA_FIT_TO_VALVE_POSE_STRUCT CManipulation::GetFitToValvePoseOption(){

    KINOVA_FIT_TO_VALVE_POSE_STRUCT fit_to_valve_pose;

    mxt_fit_to_valve_pose.lock();
    {
        fit_to_valve_pose = mstruct_fit_to_valve_pose;
    }
    mxt_fit_to_valve_pose.unlock();

    return fit_to_valve_pose;
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

void CManipulation::DataSort(std::vector<GRIPPER_DATA>& _data){

    for(unsigned int i = 0; i < _data.size(); i++){
        for(unsigned int j = i; j < _data.size(); j++){

            if(_data.at(i).y > _data.at(j).y){

                int tmp_index = 0;
                double tmp_data = 0;

                tmp_data = _data[i].y;
                _data[i].y = _data[j].y;
                _data[j].y = tmp_data;

                tmp_index = _data[i].x;
                _data[i].x = _data[j].x;
                _data[j].x = tmp_index;
            }
        }
    }
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

cv::Mat CManipulation::ValveModeling(int _valve_size, double _rotation_angle){

    // Center = 319,239
    // 212 x 212(pixel unit) = valve size on Mat
    // (212 / 2) * root(2) = 150 => Diagonal
    // _rotation_angle + 45 => Between (Center point to Diagonal point)vector and x-axis angle

    cv::Point point_1;//Left Top
    cv::Point point_2;//Right Top
    cv::Point point_3;//Right Bot
    cv::Point point_4;//Left Bot

    cv::Point point_c;//Center Point

    cv::Point point_x_axis_s(0,239);
    cv::Point point_x_axis_e(639,239);
    cv::Point point_y_axis_s(319,0);
    cv::Point point_y_axis_e(319,479);

    cv::Point point_text_size(30,50);
    cv::Point point_text_angle(30,80);

    int diagonal_distance = 150;

    double diagonal_angle_1 = RGBD_D2R * (((-1)*_rotation_angle) + 45); // For point_1
    double diagonal_angle_2 = RGBD_D2R * (90 + (((-1)*_rotation_angle) + 45)); // For point_2

    cv::Mat mat_valve = cv::Mat::zeros(480,640,CV_8UC3);

//    int scale_1 = (sin(diagonal_angle_1)) > 0 ? 1 : -1;
//    int scale_2 = (sin(diagonal_angle_2)) > 0 ? -1 : 1;

    point_c.x = 319;
    point_c.y = 239;

    point_1.x = point_c.x - (diagonal_distance) * cos(diagonal_angle_1);
    point_1.y = point_c.y + (diagonal_distance) * sin(diagonal_angle_1);

    point_2.x = point_c.x - (diagonal_distance) * cos(diagonal_angle_2);
    point_2.y = point_c.y + (diagonal_distance) * sin(diagonal_angle_2);

    point_3.x = point_c.x + (diagonal_distance) * cos(diagonal_angle_1);
    point_3.y = point_c.y - (diagonal_distance) * sin(diagonal_angle_1);

    point_4.x = point_c.x + (diagonal_distance) * cos(diagonal_angle_2);
    point_4.y = point_c.y - (diagonal_distance) * sin(diagonal_angle_2);

    cv::line(mat_valve, point_x_axis_s,point_x_axis_e,cv::Scalar(255,255,255), 3);
    cv::line(mat_valve, point_y_axis_s,point_y_axis_e,cv::Scalar(255,255,255), 3);

    cv::line(mat_valve, point_1,point_2,cv::Scalar(255,0,0), 3);
    cv::line(mat_valve, point_2,point_3,cv::Scalar(0,255,0), 3);
    cv::line(mat_valve, point_3,point_4,cv::Scalar(0,0,255), 3);
    cv::line(mat_valve, point_1,point_4,cv::Scalar(0,255,255),3);

    cv::circle(mat_valve, point_c,5,cv::Scalar(0,0,255));

    QString str_valve_size = "Valve Size: " + QString::number(_valve_size) + "mm";
    QString str_rotation_ang = "Angle: " + QString::number(_rotation_angle) + "deg";

    cv::putText(mat_valve, str_valve_size.toStdString(), point_text_size, 2, 1,cv::Scalar(255,255,255), 2);
    cv::putText(mat_valve, str_rotation_ang.toStdString(), point_text_angle, 2, 1,cv::Scalar(255,255,255), 2);

    emit SignalValveImage(mat_valve);

    return mat_valve;
}

void CManipulation::MakePanelModel(int _valve_size, int _wrench_index){

    mtx_panel_model.lock();
    {
        // 1 mm / 1 pixel
        int margin_w = 140;//0 ~ panel
        int margin_h = 140;//

        int margin_wrench = 50;//

        int valve_center = 345;//valve_location

        cv::Point point_1(margin_w,0);
        cv::Point point_2(m_mat_panel_model.cols - margin_w - 1,0);
        cv::Point point_3(m_mat_panel_model.cols - margin_w - 1,margin_h);
        cv::Point point_4(margin_w,margin_h);

        cv::Point point_panel_center(639,margin_h);

        cv::Point point_valve_s(margin_w + valve_center - (int)(_valve_size / 2)/*half of valve size*/ ,margin_h);
        cv::Point point_valve_c(margin_w + valve_center + 0/*valve center position*/,margin_h);
        cv::Point point_valve_e(margin_w + valve_center + (int)(_valve_size / 2)/*half of valve size*/ ,margin_h);

        cv::Point point_wrench_1((point_3.x - 300) + margin_wrench * 0, margin_h);
        cv::Point point_wrench_2((point_3.x - 300) + margin_wrench * 1, margin_h);
        cv::Point point_wrench_3((point_3.x - 300) + margin_wrench * 2, margin_h);
        cv::Point point_wrench_4((point_3.x - 300) + margin_wrench * 3, margin_h);
        cv::Point point_wrench_5((point_3.x - 300) + margin_wrench * 4, margin_h);
        cv::Point point_wrench_6((point_3.x - 300) + margin_wrench * 5, margin_h);

        cv::line(m_mat_panel_model, point_1,point_4,cv::Scalar(255,255,255),3);
        cv::line(m_mat_panel_model, point_2,point_3,cv::Scalar(255,255,255),3);
        cv::line(m_mat_panel_model, point_3,point_4,cv::Scalar(255,255,255),3);

        cv::circle(m_mat_panel_model, point_panel_center,5,cv::Scalar(0,0,255),5);

        cv::line(m_mat_panel_model, point_valve_s,cv::Point(point_valve_s.x, 0),cv::Scalar(255,0,0),2);
        cv::line(m_mat_panel_model, point_valve_e,cv::Point(point_valve_e.x, 0),cv::Scalar(255,0,0),2);

        cv::circle(m_mat_panel_model, point_valve_c,5,cv::Scalar(255,0,0),3);

        if(_wrench_index == 1)
            cv::circle(m_mat_panel_model, point_wrench_1,5,cv::Scalar(255,255,0),3);
        else
            cv::circle(m_mat_panel_model, point_wrench_1,5,cv::Scalar(0,255,255),3);

        if(_wrench_index == 2)
            cv::circle(m_mat_panel_model, point_wrench_2,5,cv::Scalar(255,255,0),3);
        else
            cv::circle(m_mat_panel_model, point_wrench_2,5,cv::Scalar(0,255,255),3);

        if(_wrench_index == 3)
            cv::circle(m_mat_panel_model, point_wrench_3,5,cv::Scalar(255,255,0),3);
        else
            cv::circle(m_mat_panel_model, point_wrench_3,5,cv::Scalar(0,255,255),3);

        if(_wrench_index == 4)
            cv::circle(m_mat_panel_model, point_wrench_4,5,cv::Scalar(255,255,0),3);
        else
            cv::circle(m_mat_panel_model, point_wrench_4,5,cv::Scalar(0,255,255),3);

        if(_wrench_index == 5)
            cv::circle(m_mat_panel_model, point_wrench_5,5,cv::Scalar(255,255,0),3);
        else
            cv::circle(m_mat_panel_model, point_wrench_5,5,cv::Scalar(0,255,255),3);

        if(_wrench_index == 6)
            cv::circle(m_mat_panel_model, point_wrench_6,5,cv::Scalar(255,255,0),3);
        else
            cv::circle(m_mat_panel_model, point_wrench_6,5,cv::Scalar(0,255,255),3);
    }
    mtx_panel_model.unlock();

}

cv::Mat CManipulation::GetPanelModel(){

    cv::Mat mat_panel = cv::Mat::zeros(640,1280,CV_8UC3);

    mtx_panel_model.lock();
    {
        mat_panel = m_mat_panel_model.clone();
    }
    mtx_panel_model.unlock();

    return mat_panel;
}

void CManipulation::PanelModeling(int _valve_size, int _virtical_dst/*mm*/, int _horizen_dst/*mm*/, double _angle/*deg*/){

    int margin_w = 140;//0 ~ panel
    int margin_h = 140;//

    cv::Mat mat_panel = GetPanelModel();
    cv::Point point_lrf_center(margin_w + _horizen_dst, _virtical_dst + margin_h);

    QString str_valve_size = "Valve Size: " + QString::number(_valve_size) + " mm";

    QString str_v_distance = "V Distance: " + QString::number(_virtical_dst) + " mm";
    QString str_h_distance = "H Distance: " + QString::number(_horizen_dst) + " mm";

    QString str_heading_error = "Heading Error: " + QString::number(_angle) + " Deg";

    cv::Point point_text_size(30,520);
    cv::Point point_text_v_distance(30,550);
    cv::Point point_text_h_distance(30,580);
    cv::Point point_text_heading_error(30,610);

    cv::putText(mat_panel, str_valve_size.toStdString(), point_text_size, 2, 1,cv::Scalar(255,255,255), 2);
    cv::putText(mat_panel, str_v_distance.toStdString(), point_text_v_distance, 2, 1,cv::Scalar(255,255,255), 2);
    cv::putText(mat_panel, str_h_distance.toStdString(), point_text_h_distance, 2, 1,cv::Scalar(255,255,255), 2);
    cv::putText(mat_panel, str_heading_error.toStdString(), point_text_heading_error, 2, 1,cv::Scalar(255,255,255), 2);

    cv::circle(mat_panel, point_lrf_center,10,cv::Scalar(0,255,0),5);

    cv::Point point_lrf_c_to_v_panel(point_lrf_center.x, point_lrf_center.y - _virtical_dst);
    cv::Point point_lrf_c_to_theta_panel(point_lrf_center.x + tan(RGBD_D2R*_angle) * (point_lrf_center.y - _virtical_dst),
                                         (point_lrf_center.y - 50));

    cv::arrowedLine(mat_panel, point_lrf_center, point_lrf_c_to_v_panel, cv::Scalar(0,255,0), 3);
    cv::arrowedLine(mat_panel, point_lrf_center, point_lrf_c_to_theta_panel, cv::Scalar(0,0,255), 3);

    emit SignalPanelImage(mat_panel);
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

double CManipulation::GetValveRotationRecogResult(){

    return m_valve_rotation_result;
}

void CManipulation::SetValveRotationRecogResult(double _result){
    m_valve_rotation_result = _result;
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

bool CManipulation::LRFVehicleAngleControl(){

    LRF_K_VEHICLE_ANGLE_STRUCT lrf_vehicle = GetLRFVehicleAngleOption();

    if(!lrf_vehicle.sensor_option){
        if(!mpc_vehicle->IsConnected())
            return false;
    }

    if(!mpc_lrf->IsLRFOn())
        return false;

    do{
        int direction = 0;

        double slope = 0;
        double distance = 0;

        double slope_error = 0;

        mpc_rgb_d->GetLRFInfo(slope, distance, lrf_vehicle.s_deg, lrf_vehicle.e_deg);

        lrf_vehicle.angle = slope;
        lrf_vehicle.vertical_distance = distance;

        slope_error = lrf_vehicle.desired_angle - lrf_vehicle.angle;

//        emit SignalLRFVehicleAngleStruct(lrf_vehicle);

        if(lrf_vehicle.error_boundary > fabs(slope_error)){
            break;
        }

        if(!lrf_vehicle.sensor_option){
            if(slope_error < 0){
                direction = UGV_move_left;
            }
            else{
                direction = UGV_move_right;
            }

            mpc_vehicle->Move(direction, lrf_vehicle.velocity);
        }

    }while(true);

//    mpc_vehicle->Move(UGV_move_forward, 0);

    return true;
}

bool CManipulation::LRFVehicleHorizenControl(){

    double inlier_distance = 0;

    LRF_K_VEHICLE_HORIZEN_STRUCT lrf_vehicle = GetLRFVehicleHorizenOption();
    inlier_distance = lrf_vehicle.inlier_distance;

    if(!mpc_lrf->IsLRFOn())
        return false;

    if(!lrf_vehicle.sensor_option){
        if(!mpc_vehicle->IsConnected())
            return false;
    }

//    int count = 0;

    do{
        int direction = 0;
        double current_a_inlier_deg = 0;
//        double current_a_virtual_deg = 0;

        double a_deg_boundary = 0;
//        double a_deg_virtual_boundary = 0;

        double horizen_distance = 0;

        double s_virture_deg = 0;
        double e_virture_deg = 0;

        double s_inlier_deg = 0;
        double e_inlier_deg = 0;

        mpc_rgb_d->GetHorizenDistance(inlier_distance, horizen_distance, s_inlier_deg, e_inlier_deg,
                                      s_virture_deg, e_virture_deg, lrf_vehicle.s_deg, lrf_vehicle.e_deg);

        current_a_inlier_deg = (s_inlier_deg + e_inlier_deg) / 2;
//        current_a_virtual_deg = (s_virture_deg + e_virture_deg) / 2;;

        a_deg_boundary = lrf_vehicle.desired_avr_inlier_deg - current_a_inlier_deg;
//        a_deg_virtual_boundary = lrf_vehicle.desired_avr_inlier_deg - current_a_virtual_deg;

        lrf_vehicle.horizen_distance = horizen_distance;

        lrf_vehicle.s_inlier_deg = s_inlier_deg;
        lrf_vehicle.e_inlier_deg = e_inlier_deg;

        lrf_vehicle.s_virtual_deg = s_virture_deg;
        lrf_vehicle.e_virtual_deg = e_virture_deg;

//        emit SignalLRFVehicleHorizenStruct(lrf_vehicle);

        if(lrf_vehicle.error_deg_boundary > fabs(a_deg_boundary)){
//            if(count < 10){
//                count++;
//                continue;
//            }
//            else
                break;
        }

        if(!lrf_vehicle.sensor_option){
            if(a_deg_boundary > 0){
                direction = UGV_move_forward;
            }
            else{
                direction = UGV_move_backward;
            }
            mpc_vehicle->Move(direction, lrf_vehicle.velocity);
        }

//        count = 0;s

    }while(true);

    mpc_vehicle->Move(UGV_move_forward, 0);

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
            break;
        case 2:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_2;
            break;
        case 3:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_3;
            break;
        case 4:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_4;
            break;
        case 5:
            lrf_kinova_struct.desired_inlier_deg_avr = lrf_kinova_struct.wrench_location_deg_5;
            break;
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

        if(inlier_s_deg == 0 && inlier_e_deg == 0){
            continue;
        }

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

//----------------------------------------------------------------
// LRF New Version
//----------------------------------------------------------------

bool CManipulation::LRFLocalizationOnPanel(){

    LRF_SENSING_INFO_STRUCT sensing_info = GetLRFSensingInfo();

    LOCALIZATION_INFO_ON_PANEL info;


    int v_dst = 0;
    int h_dst = 0;

    double angle = 0;

    while(sensing_info.fl_lrf_sensing){

        v_dst = 0;
        h_dst = 0;

        angle = 0;

        for(int i = 0; i < 3; i++){
            sensing_info = GetLRFSensingInfo();

            mpc_rgb_d->LocalizationOnPanel(info, sensing_info.mode, sensing_info.s_deg, sensing_info.e_deg,
                                           sensing_info.inlier_distance, sensing_info.current_dst, sensing_info.current_ang);

            if(info.vertical_dst > 500){
                info.vertical_dst = 500;
            }
            if(info.vertical_dst < 0){
                info.vertical_dst = 0;
            }

            if(info.horizen__dst > 1000){
                info.horizen__dst = 1000;
            }
            if(info.horizen__dst < 0){
                info.horizen__dst = 0;
            }

            v_dst += info.vertical_dst;
            h_dst += info.horizen__dst;
            angle += info.angle;
        }

        v_dst /= 3;
        h_dst /= 3;
        angle /= 3;

        PanelModeling(19, v_dst, h_dst, angle);
//        PanelModeling(19, info.vertical_dst, info.horizen__dst, info.angle);

        msleep(30);
    }

    return true;
}

bool CManipulation::LRFK_VCtrl(){

    LRF_K_V_CTRL_STRUCT lrf_kinova_struct = GetLRFKVerticalCtrlOption();

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    //-----------------------------------------------
    // First of all, Big Move. And Then, Precise Move
    //-----------------------------------------------
    LOCALIZATION_INFO_ON_PANEL info;

    mpc_rgb_d->LocalizationOnPanel(info, lrf_kinova_struct.lrf_info_struct.mode,
                                   lrf_kinova_struct.lrf_info_struct.s_deg,lrf_kinova_struct.lrf_info_struct.e_deg,
                                   lrf_kinova_struct.lrf_info_struct.inlier_distance);

    PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

    int error_dst =
            lrf_kinova_struct.desired_v_dst - info.vertical_dst;

    CartesianPosition position;

    double error_dst_m = ((double)error_dst) * 0.001;

    if(!lrf_kinova_struct.fl_only_sensing_moving){
        mpc_kinova->Kinova_GetCartesianPosition(position);

        if(error_dst > 0){
            position.Coordinates.X -= fabs(error_dst_m);
            mpc_kinova->KinovaDoManipulate(position, 2);
        }
        else{
            position.Coordinates.X += fabs(error_dst_m);
            mpc_kinova->KinovaDoManipulate(position, 2);
        }
    }
    msleep(1500);
    //-----------------------------------------------
    // Precise Move
    //-----------------------------------------------
    msleep(1000);
    do{
        mpc_rgb_d->LocalizationOnPanel(info, lrf_kinova_struct.lrf_info_struct.mode,
                                       lrf_kinova_struct.lrf_info_struct.s_deg,lrf_kinova_struct.lrf_info_struct.e_deg,
                                       lrf_kinova_struct.lrf_info_struct.inlier_distance);

        PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

        if(lrf_kinova_struct.fl_force_option){
            if(KinovaForceCheck(lrf_kinova_struct.force_x, lrf_kinova_struct.force_y, lrf_kinova_struct.force_z))
                return true;
        }

        error_dst = lrf_kinova_struct.desired_v_dst - info.vertical_dst;
        error_dst_m = ((double)error_dst) * 0.001;

        if(lrf_kinova_struct.error > fabs(error_dst)){
            break;
        }

        if(error_dst_m > 0){
            mpc_kinova->KinovaMoveUnitStepBw();
        }
        else if(error_dst_m < 0){
            mpc_kinova->KinovaMoveUnitStepFw();
        }

        if(lrf_kinova_struct.loop_sleep != 0)
            msleep(lrf_kinova_struct.loop_sleep/*msec*/);
    }
    while(true);

    return true;
}

bool CManipulation::LRFK_HCtrl(){

    LRF_K_H_CTRL_STRUCT lrf_kinova_struct = GetLRFKHorizenCtrlOption();

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    if(lrf_kinova_struct.desired_h_location == 0){

        switch(lrf_kinova_struct.wrench_hanger_index){

        case 1:
            lrf_kinova_struct.desired_h_location = lrf_kinova_struct.wrench_location_1;
            break;
        case 2:
            lrf_kinova_struct.desired_h_location = lrf_kinova_struct.wrench_location_2;
            break;
        case 3:
            lrf_kinova_struct.desired_h_location = lrf_kinova_struct.wrench_location_3;
            break;
        case 4:
            lrf_kinova_struct.desired_h_location = lrf_kinova_struct.wrench_location_4;
            break;
        case 5:
            lrf_kinova_struct.desired_h_location = lrf_kinova_struct.wrench_location_5;
            break;
        case 6:
            lrf_kinova_struct.desired_h_location = lrf_kinova_struct.wrench_location_6;
            break;
        }

        MakePanelModel(m_valve_size,lrf_kinova_struct.wrench_hanger_index);
    }

    //-----------------------------------------------
    // First of all, Big Move. And Then, Precise Move
    //-----------------------------------------------
    LOCALIZATION_INFO_ON_PANEL info;

    mpc_rgb_d->LocalizationOnPanel(info, lrf_kinova_struct.lrf_info_struct.mode,
                                   lrf_kinova_struct.lrf_info_struct.s_deg,lrf_kinova_struct.lrf_info_struct.e_deg,
                                   lrf_kinova_struct.lrf_info_struct.inlier_distance);

    PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

    int error_dst =
            lrf_kinova_struct.desired_h_location - info.horizen__dst;

    CartesianPosition position;
    double error_dst_m = ((double)error_dst) * 0.001;

    if(lrf_kinova_struct.fl_only_sensing_moving){
        mpc_kinova->Kinova_GetCartesianPosition(position);

        if(error_dst > 0){
            position.Coordinates.Y -= fabs(error_dst_m);
            mpc_kinova->KinovaDoManipulate(position, 2);
        }
        else{
            position.Coordinates.Y += fabs(error_dst_m);
            mpc_kinova->KinovaDoManipulate(position, 2);
        }
    }

    msleep(1500);
    //-----------------------------------------------
    // Precise Move
    //-----------------------------------------------
    do{
        mpc_rgb_d->LocalizationOnPanel(info, lrf_kinova_struct.lrf_info_struct.mode,
                                       lrf_kinova_struct.lrf_info_struct.s_deg,lrf_kinova_struct.lrf_info_struct.e_deg,
                                       lrf_kinova_struct.lrf_info_struct.inlier_distance);

        PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

        error_dst = lrf_kinova_struct.desired_h_location - info.horizen__dst;
        error_dst_m = ((double)error_dst) * 0.001;

        if(lrf_kinova_struct.error > fabs(error_dst)){
            break;
        }

        if(error_dst_m > 0){
            mpc_kinova->KinovaMoveUnitStepRi();
        }
        else if(error_dst_m < 0){
            mpc_kinova->KinovaMoveUnitStepLe();
        }

        if(lrf_kinova_struct.loop_sleep != 0)
            msleep(lrf_kinova_struct.loop_sleep/*msec*/);

    }
    while(true);

    return true;
}

bool CManipulation::LRFK_ACtrl(){

    LRF_K_A_CTRL_STRUCT lrf_kinova_struct = GetLRFKAngleCtrlOption();

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;
    if(!mpc_gripper->IsRotatorInit())
        return false;

    LOCALIZATION_INFO_ON_PANEL info;

    mpc_rgb_d->LocalizationOnPanel(info, lrf_kinova_struct.lrf_info_struct.mode,
                                   lrf_kinova_struct.lrf_info_struct.s_deg,lrf_kinova_struct.lrf_info_struct.e_deg,
                                   lrf_kinova_struct.lrf_info_struct.inlier_distance);

    PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

    do{
        int direction = 0;

        double slope_error = 0;

        mpc_rgb_d->LocalizationOnPanel(info, lrf_kinova_struct.lrf_info_struct.mode,
                                       lrf_kinova_struct.lrf_info_struct.s_deg,lrf_kinova_struct.lrf_info_struct.e_deg,
                                       lrf_kinova_struct.lrf_info_struct.inlier_distance);

        PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

        slope_error = lrf_kinova_struct.desired_angle - info.angle;

        if(lrf_kinova_struct.error > fabs(slope_error)){
            break;
        }

        if(slope_error < 0){//CCW (+)
            direction += (int)((DXL_PRO_STEP_PER_DEGREE) * lrf_kinova_struct.unit_deg);
        }
        else{//CW (-)
            direction -= (int)((DXL_PRO_STEP_PER_DEGREE) * lrf_kinova_struct.unit_deg);
        }

        mpc_gripper->RotatorGoToRelPosition(direction);
        msleep(30);

    }while(true);

    return true;
}

bool CManipulation::LRFV_ACtrl(){

    LRF_V_A_CTRL_STRUCT lrf_vehicle_struct = GetLRFVAngleCtrlOption();

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;
    if(!mpc_vehicle->IsConnected())
        return false;

    LOCALIZATION_INFO_ON_PANEL info;

    mpc_rgb_d->LocalizationOnPanel(info, lrf_vehicle_struct.lrf_info_struct.mode,
                                   lrf_vehicle_struct.lrf_info_struct.s_deg,lrf_vehicle_struct.lrf_info_struct.e_deg,
                                   lrf_vehicle_struct.lrf_info_struct.inlier_distance);

    PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

    do{
        int direction = 0;

        double slope_error = 0;

        mpc_rgb_d->LocalizationOnPanel(info, lrf_vehicle_struct.lrf_info_struct.mode,
                                       lrf_vehicle_struct.lrf_info_struct.s_deg,lrf_vehicle_struct.lrf_info_struct.e_deg,
                                       lrf_vehicle_struct.lrf_info_struct.inlier_distance);

        PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

        slope_error = lrf_vehicle_struct.desired_angle - info.angle;

        if(lrf_vehicle_struct.error > fabs(slope_error)){
            break;
        }

        if(slope_error < 0){
            direction = UGV_move_left;
        }
        else{
            direction = UGV_move_right;
        }

        mpc_vehicle->Move(direction, lrf_vehicle_struct.velocity);
        msleep(500);

    }while(true);

    return true;
}

bool CManipulation::LRFV_HCtrl(){

    LRF_V_H_CTRL_STRUCT lrf_vehicle_struct = GetLRFVHorizenCtrlOption();

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;
    if(!mpc_vehicle->IsConnected())
        return false;

    LOCALIZATION_INFO_ON_PANEL info;

    mpc_rgb_d->LocalizationOnPanel(info, lrf_vehicle_struct.lrf_info_struct.mode,
                                   lrf_vehicle_struct.lrf_info_struct.s_deg,lrf_vehicle_struct.lrf_info_struct.e_deg,
                                   lrf_vehicle_struct.lrf_info_struct.inlier_distance);

    PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

    double error_dst_m = 0;

    int error_dst =
            lrf_vehicle_struct.desired_h_location - info.horizen__dst;


    int direction = 0;

    //-----------------------------------------------
    // Precise Move
    //-----------------------------------------------
    do{
        mpc_rgb_d->LocalizationOnPanel(info, lrf_vehicle_struct.lrf_info_struct.mode,
                                       lrf_vehicle_struct.lrf_info_struct.s_deg,lrf_vehicle_struct.lrf_info_struct.e_deg,
                                       lrf_vehicle_struct.lrf_info_struct.inlier_distance);

        PanelModeling(m_valve_size, info.vertical_dst, info.horizen__dst, info.angle);

        error_dst = lrf_vehicle_struct.desired_h_location - info.horizen__dst;
        error_dst_m = ((double)error_dst) * 0.001;

        if(lrf_vehicle_struct.error > fabs(error_dst)){
            mpc_vehicle->Move(direction, 0);
            break;
        }

        if(error_dst_m > 0){
            direction = UGV_move_forward;
        }
        else if(error_dst_m < 0){
            direction = UGV_move_backward;
        }

        mpc_vehicle->Move(direction, lrf_vehicle_struct.velocity);

        if(lrf_vehicle_struct.loop_sleep != 0)
            msleep(lrf_vehicle_struct.loop_sleep/*msec*/);

    }
    while(true);

    return true;
}

bool CManipulation::LRFKinovaWrenchLocationMove(){

    LRF_KINOVA_WRENCH_LOCATION_STRUCT lrf_kinova_struct = GetLRFKinovaWrenchLocationOption();
    LRF_KINOVA_HORIZEN_CTRL_STRUCT lrf_kinova_horizen;
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
    double inlier_deg_error = 0;

    double s_virture_deg = 0;
    double e_virture_deg = 0;

    if(lrf_kinova_struct.desired_start_deg == 0){

        switch(lrf_kinova_struct.wrench_hanger_index){

        case 1:
            lrf_kinova_struct.desired_start_deg = lrf_kinova_struct.wrench_rel_location_1;
            break;
        case 2:
            lrf_kinova_struct.desired_start_deg = lrf_kinova_struct.wrench_rel_location_2;
            break;
        case 3:
            lrf_kinova_struct.desired_start_deg = lrf_kinova_struct.wrench_rel_location_3;
            break;
        case 4:
            lrf_kinova_struct.desired_start_deg = lrf_kinova_struct.wrench_rel_location_4;
            break;
        case 5:
            lrf_kinova_struct.desired_start_deg = lrf_kinova_struct.wrench_rel_location_5;
            break;
        case 6:
            lrf_kinova_struct.desired_start_deg = lrf_kinova_struct.wrench_rel_location_6;
            break;
        }
    }

    do{
        lrf_kinova_struct.inlier_deg_s_output = 0;
        lrf_kinova_struct.inlier_deg_e_output = 0;

        mpc_rgb_d->GetHorizenDistance(lrf_kinova_struct.inlier_lrf_dst, current_h_distance, inlier_s_deg, inlier_e_deg,
                                      s_virture_deg, e_virture_deg, lrf_kinova_struct.s_deg, lrf_kinova_struct.e_deg, sampling_loop);

        lrf_kinova_struct.inlier_deg_s_output = inlier_s_deg;
        lrf_kinova_struct.inlier_deg_e_output = inlier_e_deg;
        lrf_kinova_struct.current_h_distance = current_h_distance;

        inlier_deg_error = lrf_kinova_struct.desired_start_deg - inlier_s_deg;


        lrf_kinova_horizen.inlier_deg_s_output = inlier_s_deg;
        lrf_kinova_horizen.inlier_deg_e_output = inlier_e_deg;

        emit SignalLRFKinovaHorizenStruct(lrf_kinova_horizen);

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
                mpc_kinova->KinovaMoveUnitStepLe();
            }
            else if(inlier_deg_error > 0){
                mpc_kinova->KinovaMoveUnitStepRi();
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
    double position_threshold = 0.005;

    KINOVA_FORCE_CTRL_STRUCT kinova_force_ctrl = GetKinovaForceCtrlOption();

    //Mode1 : Bigger than, 2: Smaller than, 3: Range
    int mode = kinova_force_ctrl.threshold_mode;

    if(mode == 0)
        mode = 1;//Default Mode

    do{
        CartesianPosition current_pos = mpc_kinova->KinovaGetPosition();
        CartesianPosition cartesian_pos = mpc_kinova->KinovaGetCartesianForce();
        emit SignalKinovaForceVector(cartesian_pos);

        if(step_count > kinova_force_ctrl.step_count){
            return false;
        }

        if(kinova_force_ctrl.force_threshold_x != 0){

            if(position_threshold > fabs(kinova_force_ctrl.position_limit_x - current_pos.Coordinates.X))
                return false;

            if(kinova_force_ctrl.force_threshold_x > 0){
                if(mode == 1){//Bigger Than
                    if(kinova_force_ctrl.force_threshold_x < fabs(cartesian_pos.Coordinates.X)){
                        break;
                    }
                }
                else if(mode == 2){//Smaller Than
                    if(kinova_force_ctrl.force_threshold_x > fabs(cartesian_pos.Coordinates.X)){
                        break;
                    }
                }
                else if(mode == 3){//Range
                    if((kinova_force_ctrl.force_threshold_x > kinova_force_ctrl.force_range_s) &&
                            (kinova_force_ctrl.force_threshold_x > kinova_force_ctrl.force_range_e)){//Range
                        break;
                    }
                }
            }
        }

        if(kinova_force_ctrl.force_threshold_y != 0){

            if(position_threshold > fabs(kinova_force_ctrl.position_limit_y - current_pos.Coordinates.Y))
                return false;

            if(kinova_force_ctrl.force_threshold_y > 0){
                if(mode == 1){//Bigger Than
                    if(kinova_force_ctrl.force_threshold_y < fabs(cartesian_pos.Coordinates.Y)){
                        break;
                    }
                }
                else if(mode == 2){//Smaller Than
                    if(kinova_force_ctrl.force_threshold_y > fabs(cartesian_pos.Coordinates.Y)){
                        break;
                    }
                }
                else if(mode == 3){//Range
                    if((kinova_force_ctrl.force_threshold_y > kinova_force_ctrl.force_range_s) &&
                            (kinova_force_ctrl.force_threshold_y > kinova_force_ctrl.force_range_e)){//Range
                        break;
                    }
                }
            }

        }

        if(kinova_force_ctrl.force_threshold_z != 0){

            if(position_threshold > fabs(kinova_force_ctrl.position_limit_z - current_pos.Coordinates.Z))
                return false;

            if(kinova_force_ctrl.force_threshold_z > 0){
                if(mode == 1){//Bigger Than
                    if(kinova_force_ctrl.force_threshold_z < fabs(cartesian_pos.Coordinates.Z)){
                        break;
                    }
                }
                else if(mode == 2){//Smaller Than
                    if(kinova_force_ctrl.force_threshold_z > fabs(cartesian_pos.Coordinates.Z)){
                        break;
                    }
                }
                else if(mode == 3){//Range
                    if((kinova_force_ctrl.force_threshold_z > kinova_force_ctrl.force_range_s) &&
                            (kinova_force_ctrl.force_threshold_z > kinova_force_ctrl.force_range_e)){//Range
                        break;
                    }
                }
            }
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
    int check_x = 0;
    int check_y = 0;
    int check_z = 0;

    KINOVA_FORCE_CHECK_STRUCT kinova_force_check = GetKinovaForceCheckOption();

    emit SignalKinovaForceCheckOption(kinova_force_check);

    do{
        CartesianPosition cartesian_pos = mpc_kinova->KinovaGetCartesianForce();
        emit SignalKinovaForceVector(cartesian_pos);

        msleep(30);

        if(check_count > kinova_force_check.check_count){
            if(GetKinovaForceCheckOption().fl_kinova_force_sensing_option)
                continue;
            else
                break;
        }

        if(kinova_force_check.force_threshold_x != 0){
            if(fabs(cartesian_pos.Coordinates.X) > kinova_force_check.force_threshold_x){//Over Threshold X axis force
                if(GetKinovaForceCheckOption().fl_kinova_force_sensing_option)
                    continue;
                else{
                    check_x++;
//                    return true;
                }
            }
        }

        if(kinova_force_check.force_threshold_y != 0){
            if(fabs(cartesian_pos.Coordinates.Y) > kinova_force_check.force_threshold_y){//Over Threshold Y axis force
                if(GetKinovaForceCheckOption().fl_kinova_force_sensing_option)
                    continue;
                else{
                    check_y++;
//                    return true;
                }
            }
        }

        if(kinova_force_check.force_threshold_z != 0){
            if(fabs(cartesian_pos.Coordinates.Z) > kinova_force_check.force_threshold_z){//Over Threshold Z axis force
                if(GetKinovaForceCheckOption().fl_kinova_force_sensing_option)
                    continue;
                else{
                    check_z++;
                    return true;
                }
            }
        }

        check_count++;
    }
    while(true);

    if(kinova_force_check.force_threshold_x != 0){
        if(check_x >= kinova_force_check.check_threshold)
            return true;
    }
    if(kinova_force_check.force_threshold_y != 0){
        if(check_y >= kinova_force_check.check_threshold)
            return true;
    }
    if(kinova_force_check.force_threshold_z != 0){
        if(check_z >= kinova_force_check.check_threshold)
            return true;
    }

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

bool CManipulation::KinovaFitToValvePose(){

    KINOVA_FIT_TO_VALVE_POSE_STRUCT kinova_fit_to_valve_optio = GetFitToValvePoseOption();

    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    double added_x_value = 0;
    double valve_rotation_angle = 0;
    int valve_size = kinova_fit_to_valve_optio.valve_size;

    valve_rotation_angle = kinova_fit_to_valve_optio.valve_rotation_angle + 45;

    if(valve_rotation_angle == 0){
        added_x_value = (valve_size * sqrt(2.0)) / 2;

        int num_move = (int)(added_x_value / kinova_fit_to_valve_optio.move_step);

        std::cout << "x_value : " << added_x_value << std::endl;
        std::cout << "rotation angle : " << valve_rotation_angle << std::endl;

        std::cout << "Angle zero. Right num_move: " << num_move << std::endl;

        for(int i = 0; i < num_move; i++){
            mpc_kinova->KinovaMoveUnitStepRi();
            msleep(500);
        }
        return true;
    }

    else if(valve_rotation_angle < 90)
        added_x_value = ((valve_size * sqrt(2.0)) / 2) * cos(RGBD_D2R * valve_rotation_angle);
    else
        added_x_value = (-1) * ((valve_size * sqrt(2.0)) / 2) * cos(RGBD_D2R * (180 - valve_rotation_angle));

    std::cout << "x_value : " << added_x_value << std::endl;
    std::cout << "rotation angle : " << valve_rotation_angle << std::endl;

    if(added_x_value > 0){
        int num_move = (int)(added_x_value / kinova_fit_to_valve_optio.move_step);
        std::cout << "Left num_move: " << num_move << std::endl;

        for(int i = 0; i < num_move; i++){
            mpc_kinova->KinovaMoveUnitStepLe();
            msleep(500);
        }
    }

    if(added_x_value < 0){
        int num_move = (int)(fabs(added_x_value) / kinova_fit_to_valve_optio.move_step);
        std::cout << "Right num_move: " << num_move << std::endl;

        for(int i = 0; i < num_move; i++){
            mpc_kinova->KinovaMoveUnitStepRi();
            msleep(500);
        }
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

    GRIPPER_STATUS gripper_status;

    QVector<double> gripper_data_x;
    QVector<double> gripper_data_y;

    GRIPPER_DATA gripper_data;
    std::vector<GRIPPER_DATA> vec_gripper_data;

    vec_gripper_data.clear();

    CartesianPosition current_pose;
    current_pose = mpc_kinova->KinovaGetPosition();

    // -1: Add Graph
    emit SignalValveSizeData(gripper_data_x, gripper_data_y, -1);

    double diff_pose = 0;
    int diff_pose_1 = 0;
    int diff_pose_2 = 0;

    double roll_angle = 0;

    bool fl_gripper_1_not_reach = false;
    bool fl_gripper_2_not_reach = false;

    int grasp_trial = gripper_kinova_valve_recog.trial;

    for(int i = 0; i < grasp_trial; i++){

        mpc_gripper->GripperGoToThePositionLoadCheck(grasp_pose_1, grasp_pose_2, force_threshold);

        gripper_status = mpc_gripper->GetGripperStatus();

        if(gripper_status.present_pose_1 < grasp_pose_1 + 5){
            fl_gripper_1_not_reach = true;
        }

        if(gripper_status.present_pose_2 < grasp_pose_2 + 5){
            fl_gripper_2_not_reach = true;
        }

        if(fl_gripper_1_not_reach || fl_gripper_2_not_reach ){

            double y_step = 0;
            double z_step = 0;

            if(!fl_gripper_1_not_reach && fl_gripper_2_not_reach){
                y_step = 2*VEL * cos(roll_angle);
                z_step = 2*VEL * sin(roll_angle);
            }
            else if(fl_gripper_1_not_reach && !fl_gripper_2_not_reach){
                y_step = (-1) * 2*VEL * cos(roll_angle);
                z_step = (-1) * 2*VEL * sin(roll_angle);
            }
            else if(fl_gripper_1_not_reach && fl_gripper_2_not_reach){
                std::cout << "1 and 2 not reched to valve" << std::endl;
            }

            mpc_gripper->GripperGoToThePositionLoadCheck(release_pose_1, release_pose_2, -2);
            msleep(500);

            mpc_kinova->KinovaMoveUnitStep(0, y_step, z_step);
            msleep(500);

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

        gripper_data.x = i;
        gripper_data.y = diff_pose;

        vec_gripper_data.push_back(gripper_data);

        emit SignalValveSizeData(gripper_data_x, gripper_data_y, m_valve_size_graph_index);

        mpc_gripper->GripperGoToThePositionLoadCheck(release_pose_1, release_pose_2, force_threshold);

        // KINOVA_PI / grasp_trial => if(i == grasp_trial) => Half Rotation
        if((grasp_trial/2) > (i) ){
            roll_angle += (KINOVA_PI / grasp_trial);
            current_pose.Coordinates.ThetaZ += (KINOVA_PI / grasp_trial);
        }
        else{
            roll_angle -= (KINOVA_PI / grasp_trial);
            current_pose.Coordinates.ThetaZ -= (KINOVA_PI / grasp_trial);
        }
        mpc_kinova->KinovaDoManipulate(current_pose, 3);

        msleep(100);
    };

    m_valve_size_graph_index++;

    mpc_gripper->GripperGoToThePositionLoadCheck(2300, 2300, -2);

    msleep(1000);

    if(vec_gripper_data.size() == 0)
        return false;

    //Find Min Diff & Valve Rotation Angle
    DataSort(vec_gripper_data);

    //Minimum Diff Index * (Step Rotation Angle)
    double rotation_angle =
            (vec_gripper_data.at(0).x * (KINOVA_PI / grasp_trial) * (180 / KINOVA_PI) /*Rad to Deg*/);

    //    gripper_kinova_valve_recog.rotation_angle =
//    (vec_gripper_data.at(0).x * (KINOVA_PI / grasp_trial) * (180 / KINOVA_PI) /*Rad to Deg*/);

    gripper_kinova_valve_recog.rotation_angle = rotation_angle;

    //Find Valve Size
    int valve_size_recog = DataAnalisys(DataSort(gripper_data_y));

    gripper_kinova_valve_recog.valve_size = valve_size_recog;

    std::cout << "Rotation Angle: " << rotation_angle << std::endl;

    if(rotation_angle > 90){
        rotation_angle -= 180;
        rotation_angle *= (-1);
    }

    SetValveSizeRecogResult(valve_size_recog);
    SetValveRotationRecogResult(rotation_angle);

    cv::Mat valve_model;
    valve_model = ValveModeling(gripper_kinova_valve_recog.valve_size, gripper_kinova_valve_recog.rotation_angle);

    SetManipulationOption(gripper_kinova_valve_recog);

    std::cout << "Valve Size: " << valve_size_recog << std::endl;
    std::cout << "Rotation Angle: " << gripper_kinova_valve_recog.rotation_angle << std::endl;

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

    WRENCH_RECOGNITION wrench_recognition = GetWrenchRecognitionOption();

    if(!mpc_camera->isRunning())
        return false;

    cv::Mat camera_image;
    vector<vector<int>> bb_info;
    vector<int> bb_info_x_ori;
    vector<int> bb_info_x_sort;
    vector<int> bb_info_y_ori;
    vector<int> bb_info_y_sort;


    if(!mpc_camera->GetCameraImage(camera_image))
        return false;

//    mpc_rgb_d->SSD~~~
    bb_info = mpc_ssd->GetSSDImage(camera_image);

    int loop_count = 0;

    while(bb_info.size() != (unsigned int)wrench_recognition.num_of_wrench)
    {
        if(loop_count > wrench_recognition.loop_count){
            wrench_recognition.wrench_location = -1;
            SetManipulationOption(wrench_recognition);
            return false;
        }
        mpc_camera->GetCameraImage(camera_image);
        msleep(300);

        bb_info = mpc_ssd->GetSSDImage(camera_image);
        msleep(300);

        loop_count++;
    }

    for(vector<vector<int>>::iterator it = bb_info.begin();it < bb_info.end();++it)
    {
        bb_info_x_ori.push_back((*it).at(0));
        bb_info_y_ori.push_back((*it).at(1));
    }


    bb_info_y_sort = bb_info_y_ori;
    std::sort(bb_info_y_sort.begin(),bb_info_y_sort.end(),std::greater<int>());
    int wrench_index_app;

//    wrench_recognition.num_of_wrench

    switch(wrench_recognition.valve_size)
    {
    case 24:
        wrench_index_app = 0;
        break;
    case 22:
        wrench_index_app = 1;
        break;
    case 19:
        wrench_index_app = 2;
        break;
    case 18:
        wrench_index_app = 3;
        break;
    case 17:
        wrench_index_app = 4;
        break;
    case 16:
        wrench_index_app = 5;
        break;
    default:
        std::cout<<"Unclassified rench index"<<std::endl;
        break;
    }


    vector<int>::iterator it_x,it_y;
    it_y = std::find(bb_info_y_ori.begin(),bb_info_y_ori.end(),bb_info_y_sort.at(wrench_index_app));
    int index_of_x = std::distance(bb_info_y_ori.begin(),it_y);

    int rench_x_loc = bb_info_x_ori.at(index_of_x);

    bb_info_x_sort = bb_info_x_ori;

    std::sort(bb_info_x_sort.begin(),bb_info_x_sort.end());

    it_x = std::find(bb_info_x_sort.begin(),bb_info_x_sort.end(),rench_x_loc);

    wrench_recognition.wrench_location = std::distance(bb_info_x_sort.begin(),it_x) + 1;
    SetManipulationOption(wrench_recognition);

    std::cout << "wrench_location : " << wrench_recognition.wrench_location << std::endl;
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

    /*New LRF Kinova Control*/
    case MANIPUL_INX_LRF_K_VERTIVAL_CTRL:
        LRFK_VCtrl();
        break;
    case MANIPUL_INX_LRF_K_HORIZEN_CTRL:
        LRFK_HCtrl();
        break;
    case MANIPUL_INX_LRF_K_ANGLE_CTRL:
        LRFK_ACtrl();
        break;

    case MANIPUL_INX_LRF_V_HORIZEN_CTRL:
        LRFV_HCtrl();
        break;

    /*Old LRF Kinova Control*/
    case MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL:
        LRFKinovaVerticalControl();
        break;
    case MANIPUL_INX_LRF_KINOVA_HORIZEN_CTRL:
        LRFKinovaHorizenControl();
        break;
    case MANIPUL_INX_LRF_K_VEHICLE_HORIZEN_CTRL:
        LRFVehicleHorizenControl();
        break;
    case MANIPUL_INX_LRF_K_VEHICLE_ANGLE_CTRL:
        LRFVehicleAngleControl();
        break;
    case MANIPUL_INX_KINOVA_FORCE_CLRL:
        SetMainFunctionResult(KinovaForceCtrl());
        break;
    case MANIPUL_INX_KINOVA_FORCE_CHECK:
        SetMainFunctionResult(KinovaForceCheck());
        break;
    case MANIPUL_INX_KINOVA_FIT_TO_VALVE:
        KinovaFitToValvePose();
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
    case MANIPUL_INX_LRF_KINOVA_WRENCH_LOCATION:
        LRFKinovaWrenchLocationMove();
        break;
    case MANIPUL_INX_KINOVA_ALIGN_TO_PANEL:
        KinovaAlignPanel();
        break;
    case SENSING_LRF_PANEL_LOCALIZATION:
        LRFLocalizationOnPanel();
    default:
        break;

    }
}

