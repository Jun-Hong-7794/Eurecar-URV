#include "CManipulation.h"

CManipulation::CManipulation(){

}

CManipulation::~CManipulation(){

    if(this->isRunning())
        this->terminate();
}

CManipulation::CManipulation(CLRF *_p_lrf, CCamera *_p_camera, CKinova *_p_kinova, CVehicle *_p_vehicle, CVelodyne *_p_velodyne, CGripper* _p_gripper){

    mpc_lrf = _p_lrf;
    mpc_camera = _p_camera;
    mpc_kinova = _p_kinova;
    mpc_vehicle = _p_vehicle;
    mpc_velodyne = _p_velodyne;
    mpc_gripper = _p_gripper;

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

    if(_fnc_index_ == MANIPUL_INX_LRF_KINOVA){
        m_main_fnc_index = MANIPUL_INX_LRF_KINOVA;
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
    else if(_fnc_index_ == MANIPUL_INX_LRF_VEHICLE){
        m_main_fnc_index = MANIPUL_INX_LRF_VEHICLE;
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

void CManipulation::SetManipulationOption(LRF_KINOVA_STRUCT _lrf_kinova_option){

    mxt_lrf_kinova.lock();
    {
        mstruct_lrf_kinova = _lrf_kinova_option;
    }
    mxt_lrf_kinova.unlock();
}

LRF_KINOVA_STRUCT CManipulation::GetLRFKinovaOption(){

    LRF_KINOVA_STRUCT lrf_kinova_struct;

    mxt_lrf_kinova .lock();
    {
        lrf_kinova_struct = mstruct_lrf_kinova;
    }
    mxt_lrf_kinova.unlock();

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

void CManipulation::SetManipulationOption(LRF_VEHICLE_STRUCT _manipulation_option){

    mxt_lrf_vehicle.lock();
    {
        mstruct_lrf_vehicle = _manipulation_option;
    }
    mxt_lrf_vehicle.unlock();
}

LRF_VEHICLE_STRUCT CManipulation::GetLRFVehicleOption(){

    LRF_VEHICLE_STRUCT lrf_vehicle;

    mxt_lrf_vehicle.lock();
    {
        lrf_vehicle = mstruct_lrf_vehicle;
    }
    mxt_lrf_vehicle.unlock();

    return lrf_vehicle;
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
bool CManipulation::LRFKinovaDepthControl(){

    if(!mpc_lrf->IsLRFOn())
        return false;
    if(!mpc_kinova->IsKinovaInitialized())
        return false;

    double slope = 0;
    double current_distance = 0;

    LRF_KINOVA_STRUCT lrf_kinova_struct = GetLRFKinovaOption();

    do{
        mpc_rgb_d->GetLRFInfo(slope, current_distance, lrf_kinova_struct.s_deg, lrf_kinova_struct.e_deg);

        double current_error = lrf_kinova_struct.desired_distance - current_distance;

        if(fabs(current_error) < lrf_kinova_struct.error){
            break;
        }

        if(current_error < 0){
            mpc_kinova->KinovaMoveUnitStepFw();
        }
        else if(current_error > 0){
            mpc_kinova->KinovaMoveUnitStepBw();
        }

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

    kinova_pose.Coordinates.ThetaX = kinova_manipulate.roll;
    kinova_pose.Coordinates.ThetaY = kinova_manipulate.pitch;
    kinova_pose.Coordinates.ThetaZ = kinova_manipulate.yaw;

    mpc_kinova->KinovaDoManipulate(kinova_pose, 2, kinova_manipulate.forece_threshold);

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

bool CManipulation::LRFVehicleControl(){

    if(!mpc_lrf->IsLRFOn())
        return false;

    double inlier_distance = 0;
    double horizen_distance = 0;
    double s_inlier_deg = 0;
    double e_inlier_deg = 0;

    LRF_VEHICLE_STRUCT lrf_vehicle = GetLRFVehicleOption();
    inlier_distance = lrf_vehicle.inlier_distance;

    do{
        mpc_rgb_d->GetHorizenDistance(inlier_distance, horizen_distance, s_inlier_deg, e_inlier_deg);

        lrf_vehicle.horizen_distance = horizen_distance;

        lrf_vehicle.s_inlier_deg = s_inlier_deg;
        lrf_vehicle.e_inlier_deg = e_inlier_deg;

        emit SignalLRFHorizentDistance(lrf_vehicle);
    }while(true);

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CManipulation::run(){

    switch (m_main_fnc_index){

    case MANIPUL_INX_LRF_KINOVA:
        LRFKinovaDepthControl();
        break;
    case MANIPUL_INX_KINOVA_FORCE_CLRL:
        KinovaForceCtrl();
        break;
    case MANIPUL_INX_GRIPPER_FORCE_CLRL:
        GripperForceCtrl();
        break;
    case MANIPUL_INX_LRF_VEHICLE:
        LRFVehicleControl();
        break;
    case MANIPUL_INX_KINOVA_MANIPULATE:
        KinovaDoManipulate();
        break;
    default:
        break;

    }
}

