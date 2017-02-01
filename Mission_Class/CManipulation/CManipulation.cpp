#include "CManipulation.h"

CManipulation::CManipulation(){

}

CManipulation::~CManipulation(){

    if(this->isRunning())
        this->terminate();
}

CManipulation::CManipulation(CLRF *_p_mani_lrf, CCamera *_p_camera, CKinova *_p_kinova, CVehicle *_p_vehicle, CVelodyne *_p_velodyne, CGripper* _p_gripper){

    mpc_lrf = _p_mani_lrf;
    mpc_camera = _p_camera;
    mpc_kinova = _p_kinova;
    mpc_vehicle = _p_vehicle;
    mpc_velodyne = _p_velodyne;
    mpc_gripper = _p_gripper;

    mpc_rgb_d = new CRGBD(_p_camera, _p_mani_lrf);

    connect(mpc_kinova, SIGNAL(SignalKinovaPosition(CartesianPosition)), this, SIGNAL(SignalKinovaPosition(CartesianPosition)));
    connect(mpc_rgb_d, SIGNAL(SignalLRFMapImage(cv::Mat)), this, SIGNAL(SignalLRFImage(cv::Mat)));
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

//End Effector

bool CManipulation::InitGripper(char* _device_port){

    if(!mpc_gripper->InitDynamixel(_device_port))
        return false;

    if(!mpc_gripper->IsDmxTorqueOn())
        mpc_gripper->DynamixelTorque(true);

    return true;
}

bool CManipulation::CloseGripper(){

    mpc_gripper->CloseDynamixel();

    return true;
}

bool CManipulation::GripperGoRelPose(double _deg){

    if(!mpc_gripper->DynamixelGoToRelPosition(_deg))
        return false;

    return true;
}

bool CManipulation::GripperGoThePose(double _deg){

    if(!mpc_gripper->DynamixelGoToThePosition(_deg))
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
    else if(_fnc_index_ == MANIPUL_INX_GRIPPER_FORCE_CLRL){
        m_main_fnc_index = MANIPUL_INX_GRIPPER_FORCE_CLRL;
        this->start();

        return true;
    }
    else if(_fnc_index_ == MANIPUL_INX_KINOVA_MANIPULATE){
        m_main_fnc_index = MANIPUL_INX_KINOVA_MANIPULATE;
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

//----------------------------------------------------------------
// Main Function
//----------------------------------------------------------------
bool CManipulation::LRFKinovaVerticalControl(){

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;


    LRF_KINOVA_VERTICAL_CTRL_STRUCT lrf_kinova_struct = GetLRFKinovaVerticalOption();

    do{
        double slope = 0;
        double current_distance = 0;

        mpc_rgb_d->GetLRFInfo(slope, current_distance, lrf_kinova_struct.s_deg, lrf_kinova_struct.e_deg, lrf_kinova_struct.inlier_lrf_dst);

        double current_error = lrf_kinova_struct.desired_distance - current_distance;/*mm*/

        std::cout << "C-LRF Data : " << current_distance<<std::endl;
        std::cout << "D-LRF Data : " << lrf_kinova_struct.desired_distance<<std::endl;

        if(fabs(current_error) < lrf_kinova_struct.error){
            break;
        }

        if(current_error < 0){
            mpc_kinova->KinovaMoveUnitStepFw();
        }
        else if(current_error > 0){
            mpc_kinova->KinovaMoveUnitStepBw();
        }

        if(lrf_kinova_struct.loop_sleep != 0)
            msleep(lrf_kinova_struct.loop_sleep/*msec*/);
    }
    while(true);

    return true;
}

bool CManipulation::LRFKinovaHorizenControl(){

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;


    LRF_KINOVA_HORIZEN_CTRL_STRUCT lrf_kinova_struct = GetLRFKinovaHorizenOption();

    do{
        double s_inlier_deg = 0;
        double e_inlier_deg = 0;
        double current_h_distance = 0;
        double inlier_deg_avr = 0;
        double inlier_deg_error = 0;

        mpc_rgb_d->GetHorizenDistance(lrf_kinova_struct.inlier_lrf_dst, current_h_distance, s_inlier_deg, e_inlier_deg);

        inlier_deg_avr = (s_inlier_deg + e_inlier_deg) / 2;
        inlier_deg_error = inlier_deg_avr - lrf_kinova_struct.desired_inlier_deg_avr;

        if(fabs(inlier_deg_error) < lrf_kinova_struct.error){
            break;
        }

        if(inlier_deg_error < 0){
            mpc_kinova->KinovaMoveUnitStepLe();
        }
        else if(inlier_deg_error > 0){
            mpc_kinova->KinovaMoveUnitStepRi();
        }

        if(lrf_kinova_struct.loop_sleep != 0)
            msleep(lrf_kinova_struct.loop_sleep/*msec*/);

    }
    while(true);

    return true;
}

bool CManipulation::KinovaForceCtrl(){

    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    int step_count = 0;

    KINOVA_FORCE_CTRL_STRUCT kinova_force_ctrl = GetKinovaForceCtrlOption();

    do{
        CartesianPosition cartesian_pos = mpc_kinova->KinovaGetCartesianForce();
        emit SignalKinovaForceVector(cartesian_pos);

        if(step_count > kinova_force_ctrl.step_count)
            break;

        if(fabs(cartesian_pos.Coordinates.X) > kinova_force_ctrl.forece_threshold){//Over Threshold X axis force
            break;
        }
        if(fabs(cartesian_pos.Coordinates.Y) > kinova_force_ctrl.forece_threshold){//Over Threshold Y axis force
            break;
        }
        if(fabs(cartesian_pos.Coordinates.Z) > kinova_force_ctrl.forece_threshold){//Over Threshold Z axis force
            break;
        }

        mpc_kinova->KinovaMoveUnitStep(kinova_force_ctrl.move_step_x, kinova_force_ctrl.move_step_y, kinova_force_ctrl.move_step_z);

        step_count++;
    }
    while(true);

    return true;
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

//    mpc_kinova->KinovaDoManipulate(kinova_pose, kinova_manipulate.forece_threshold);
    mpc_kinova->KinovaDoManipulate(kinova_pose, 2);

    return true;
}

bool CManipulation::GripperForceCtrl(){

    if(!mpc_gripper->IsDmxInit())
        return false;
    if(!mpc_gripper->IsDmxTorqueOn())
        return false;

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_ctrl;

    gripper_force_ctrl = GetGripperForceCtrlOption();

    mpc_gripper->DynamixelGoToThePositionUsingLoad(gripper_force_ctrl.bend_deg, gripper_force_ctrl.forece_threshold);

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CManipulation::run(){

    switch (m_main_fnc_index){

    case MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL:
        LRFKinovaVerticalControl();
        break;
    case MANIPUL_INX_LRF_KINOVA_HORIZEN_CTRL:
        LRFKinovaHorizenControl();
        break;
    case MANIPUL_INX_KINOVA_FORCE_CLRL:
        KinovaForceCtrl();
        break;
    case MANIPUL_INX_GRIPPER_FORCE_CLRL:
        GripperForceCtrl();
        break;
    case MANIPUL_INX_KINOVA_MANIPULATE:
        KinovaDoManipulate();
        break;
    default:
        break;

    }
}

