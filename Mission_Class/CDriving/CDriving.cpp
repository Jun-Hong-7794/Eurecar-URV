#include "CDriving.h"

CDriving::CDriving(){

}

CDriving::CDriving(CGPS* _p_gps, CLRF* _p_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne){

    mpc_gps = _p_gps;
    mpc_drive_lrf = _p_lrf;
    mpc_camera = _p_camera;
    mpc_kinova = _p_kinova;
    mpc_vehicle = _p_vehicle;
    mpc_velodyne = _p_velodyne;

    mpc_rgb_d = new CRGBD(_p_lrf);

    connect(mpc_velodyne,SIGNAL(SignalVelodyneParser(bool)),this,SIGNAL(SignalVelodyneParser(bool)));
    connect(mpc_rgb_d,SIGNAL(SignalLRFMapImage(cv::Mat)),this,SIGNAL(SignalLRFMapImage(cv::Mat)));
}

CDriving::~CDriving(){

    if(this->isRunning()){
        if(m_main_fnc_index == DRIVE_INX_DRIVE_TO_PANEL){
            DRIVING_STRUCT driving_struct;
            driving_struct.direction = 1;
            driving_struct.velocity = 0;
            driving_struct.driving_mission = false;
            SetDrivingOption(driving_struct);
        }

        else if(m_main_fnc_index == DRIVE_INX_PARKING__PANEL){

        }
    }
}

//----------------------------------------------------------------
// Device Function
//----------------------------------------------------------------
bool CDriving::ConnectVehicle(){

    if(!mpc_vehicle->Connect()){
        std::cout << "Fail to Connection Vehicle" << std::endl;
        return false;
    }

    return true;
}

bool CDriving::ConnectVelodyne(){

    if(!mpc_velodyne->IsVelodyneConneted()){
        if(mpc_velodyne->SetVelodyneThread(true))
            return true;
        else
            return false;
    }
    else
        return false;
}

bool CDriving::CloseVelodyne(){

    if(mpc_velodyne->SetVelodyneThread(false))
        return true;
    else
        return false;
}

bool CDriving::IsVelodyneConnected(){
    return mpc_velodyne->IsVelodyneConneted();
}

void CDriving::PCLInit(){
    mpc_velodyne->PCLInitialize();

    std::string str_ugv3dmodel = "/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/ugv_3d_model.STL";
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(str_ugv3dmodel,mesh);
    mpc_velodyne->GetPCL()->viewer->addPolygonMesh(mesh);
}

CPCL* CDriving::GetPCL(){

    return mpc_velodyne->GetPCL();
}

int CDriving::GetPanelHeadingError(){
    std::vector<double> panel_loc;

    panel_loc = mpc_velodyne->GetPanelCenterLoc();

    double heading_error;

    double error_margin = 3.141592/180.0*(10.0);

    if((panel_loc.at(0) == 0) | (panel_loc.at(1) == 0))
    {
        return -2;
    }
    else
    {
        heading_error = std::atan(panel_loc.at(1)/(-panel_loc.at(0))) -0.06;

        if((heading_error >  0) &&  (heading_error > error_margin))
        {
            return 1;
        }
        else if((heading_error <  0) &&  (heading_error < error_margin))
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }

}


//----------------------------------------------------------------
// Option Function
//----------------------------------------------------------------

bool CDriving::SelectMainFunction(int _fnc_index_){

    if(this->isRunning()){ //A thread is Running?
        return false;
    }

    if(_fnc_index_ == DRIVE_INX_DRIVE_TO_PANEL){
        m_main_fnc_index = DRIVE_INX_DRIVE_TO_PANEL;
        this->start();

        return true;
    }
    else if(_fnc_index_ == DRIVE_INX_PARKING__PANEL){
        m_main_fnc_index = DRIVE_INX_PARKING__PANEL;
        this->start();

        return true;
    }
    else if(_fnc_index_ == DRIVE_INX_LRF_VEHICLE_ANGLE){
        m_main_fnc_index = DRIVE_INX_LRF_VEHICLE_ANGLE;
        this->start();

        return true;
    }
    else if(_fnc_index_ == DRIVE_INX_LRF_VEHICLE_HORIZEN){
        m_main_fnc_index = DRIVE_INX_LRF_VEHICLE_HORIZEN;
        this->start();

        return true;
    }
    else
        return false;
}

void CDriving::SetDrivingOption(DRIVING_STRUCT _driving_option){

    mtx_driving_struct.lock();
    {
        mstruct_driving = _driving_option;
    }
    mtx_driving_struct.unlock();
}

void CDriving::SetParkingOption(PARKING_STRUCT _parking_option){

    mtx_parking_struct.lock();
    {
        mstruct_parking = _parking_option;
    }
    mtx_parking_struct.unlock();
}

DRIVING_STRUCT CDriving::GetDrivingOption(){

    DRIVING_STRUCT driving_struct;

    mtx_driving_struct.lock();
    {
        driving_struct = mstruct_driving;
    }
    mtx_driving_struct.unlock();

    return driving_struct;
}

PARKING_STRUCT CDriving::GetParkingOption(){

    PARKING_STRUCT parking_struct;

    mtx_parking_struct.lock();
    {
        parking_struct = mstruct_parking;
    }
    mtx_parking_struct.unlock();

    return parking_struct;
}

void CDriving::SetManipulationOption(LRF_VEHICLE_HORIZEN_STRUCT _manipulation_option){

    mxt_lrf_vehicle.lock();
    {
        mstruct_lrf_vehicle = _manipulation_option;
    }
    mxt_lrf_vehicle.unlock();
}

LRF_VEHICLE_HORIZEN_STRUCT CDriving::GetLRFVehicleHorizenOption(){

    LRF_VEHICLE_HORIZEN_STRUCT lrf_vehicle;

    mxt_lrf_vehicle.lock();
    {
        lrf_vehicle = mstruct_lrf_vehicle;
    }
    mxt_lrf_vehicle.unlock();

    return lrf_vehicle;
}

void CDriving::SetManipulationOption(LRF_VEHICLE_ANGLE_STRUCT _manipulation_option){

    mxt_lrf_vehicle_angle.lock();
    {
        mstruct_lrf_vehicle_angle = _manipulation_option;
    }
    mxt_lrf_vehicle_angle.unlock();
}

LRF_VEHICLE_ANGLE_STRUCT CDriving::GetLRFVehicleAngleOption(){

    LRF_VEHICLE_ANGLE_STRUCT lrf_vehicle;

    mxt_lrf_vehicle_angle.lock();
    {
        lrf_vehicle = mstruct_lrf_vehicle_angle;
    }
    mxt_lrf_vehicle_angle.unlock();

    return lrf_vehicle;
}


//----------------------------------------------------------------
// Main Function
//----------------------------------------------------------------

bool CDriving::DriveToPanel(){

    DRIVING_STRUCT driving_struct;

    if(!mpc_vehicle->Connect()){
        std::cout << "Fail to Connection Vehicle" << std::endl;
        return false;
    }

    do{
        int heading_control_flag = GetPanelHeadingError();
        vector<double> panel_loc_info = mpc_velodyne->GetPanelCenterLoc();

        double distance = panel_loc_info.at(2);
        int test_vel;
        if(distance < 0.6)
            test_vel =0;
        else if(distance >= 0.6)
            test_vel = 65;
        else
            test_vel =0;

        switch(heading_control_flag)
        {
        case -1: // turn left
            driving_struct.direction = UGV_move_left;
            driving_struct.velocity =75;
            break;
        case 1: // turn right
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity =75;
            break;
        case 0: // Go straight
            driving_struct.direction = UGV_move_forward;
            driving_struct.velocity =test_vel;
            break;
        default:

            break;
        }

        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    }while(driving_struct.driving_mission);

    return true;
}

bool CDriving::ParkingFrontPanel(){

    while(true){

    }

    return true;
}

bool CDriving::LRFVehicleHorizenControl(){

    double inlier_distance = 0;

    LRF_VEHICLE_HORIZEN_STRUCT lrf_vehicle = GetLRFVehicleHorizenOption();
    inlier_distance = lrf_vehicle.inlier_distance;

    if(!mpc_drive_lrf->IsLRFOn())
        return false;

    if(!lrf_vehicle.sensor_option){
        if(!mpc_vehicle->IsConnected())
            return false;
    }

    do{
        int direction = 0;
        double current_a_inlier_deg = 0;
        double a_deg_boundary = 0;

        double horizen_distance = 0;
        double s_inlier_deg = 0;
        double e_inlier_deg = 0;

        mpc_rgb_d->GetHorizenDistance(inlier_distance, horizen_distance, s_inlier_deg, e_inlier_deg);

        current_a_inlier_deg = (s_inlier_deg + e_inlier_deg) / 2;
        a_deg_boundary = lrf_vehicle.desired_avr_inlier_deg - current_a_inlier_deg;

        lrf_vehicle.horizen_distance = horizen_distance;

        lrf_vehicle.s_inlier_deg = s_inlier_deg;
        lrf_vehicle.e_inlier_deg = e_inlier_deg;

        emit SignalLRFVehicleHorizenStruct(lrf_vehicle);

        if(lrf_vehicle.error_deg_boundary > fabs(a_deg_boundary)){
            break;
        }

        if(!lrf_vehicle.sensor_option){
            if(a_deg_boundary < 0){
                direction = UGV_move_forward;
            }
            else{
                direction = UGV_move_backward;
            }
            mpc_vehicle->Move(direction, lrf_vehicle.velocity);
        }


    }while(true);

    mpc_vehicle->Move(UGV_move_forward, 0);

    return true;
}

bool CDriving::LRFVehicleAngleControl(){

    LRF_VEHICLE_ANGLE_STRUCT lrf_vehicle = GetLRFVehicleAngleOption();

    if(!lrf_vehicle.sensor_option){
        if(!mpc_vehicle->IsConnected())
            return false;
    }

    if(!mpc_drive_lrf->IsLRFOn())
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

        emit SignalLRFVehicleAngleStruct(lrf_vehicle);

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

    mpc_vehicle->Move(UGV_move_forward, 0);

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CDriving::run(){

    switch (m_main_fnc_index) {

    case DRIVE_INX_DRIVE_TO_PANEL:
        DriveToPanel();
        break;

    case DRIVE_INX_PARKING__PANEL:
        ParkingFrontPanel();
        break;
    case DRIVE_INX_LRF_VEHICLE_ANGLE:
        LRFVehicleAngleControl();
        break;
    case DRIVE_INX_LRF_VEHICLE_HORIZEN:
        LRFVehicleHorizenControl();
        break;

    default:
        break;

    }
}
