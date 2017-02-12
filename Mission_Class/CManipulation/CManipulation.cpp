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

//----------------------------------------------------------------
// Main Function Result
//----------------------------------------------------------------
void CManipulation::SetMainFunctionResult(bool _result){

    fl_main_fnc_result = _result;
}

bool CManipulation::GetMainFunctionResult(){

    return fl_main_fnc_result;
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

    int inlier_error = gripper_kinova_valve_recog.inlier_error;

    double unit_rotation_angle = gripper_kinova_valve_recog.unit_rotation_angle;

    GRIPPER_STATUS gripper_status;

    double diff_pose = 0;
    double diff_min_pose = 3000/*No meaning, Just Big number*/;
    double diff_pose_1 = 0;
    double diff_pose_2 = 0;

    int count = 0;

    do{

        mpc_gripper->GripperGoToThePositionLoadCheck(grasp_pose_1, grasp_pose_2, force_threshold);

        gripper_status = mpc_gripper->GetGripperStatus();

        diff_pose_1 = fabs(gripper_status.present_pose_1 - grasp_pose_1);
        diff_pose_2 = fabs(gripper_status.present_pose_2 - grasp_pose_2);

        diff_pose = fabs(diff_pose_1 + diff_pose_2);

        if(diff_min_pose > diff_pose){
            diff_min_pose = diff_pose;
            if(count > 5)
                break;
            count++;
        }
        else if(diff_min_pose > fabs(diff_pose - inlier_error)){//if inlier Error
            if(count > 5)
                break;
            count++;
        }
        else{
            unit_rotation_angle = (-1) * unit_rotation_angle;
        }

        mpc_gripper->GripperGoToThePositionLoadCheck(release_pose_1, release_pose_2, -2);

        mpc_kinova->KinovaMoveUnitStep(0,0,0,0,0,unit_rotation_angle);

    }while(true);

    SetManipulationOption(gripper_kinova_valve_recog);

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
    case MANIPUL_INX_KINOVA_MANIPULATE:
        KinovaDoManipulate();
        break;
    case MANIPUL_INX_KINOVA_ROTATE_VALVE:
        KinovaRotateValveMotion();
        break;
    default:
        break;

    }
}

