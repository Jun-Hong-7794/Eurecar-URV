#include "CDriving.h"

vector<vector<double>> lrf_panelpoint = {{0.5,0.0},{0.0,0.375}};
vector<vector<double>> lrf_waypoint = {{1.5,1.425}, {0.0,1.425}, {-1.5,1.425}, {-1.5,0.0}};
double lrf_waypoint_update_thres = 0.2;

double parking_dist_margin = 0.2;

// driving tuning parameter---------------
int drive_left = 100;
int drive_differ_left = 100;
int drive_right = 150;
int drive_differ_right = 100;

int parking_left = 100;
int parking_differ_left = 130;
int parking_right = 150;
int parking_differ_right = 130;
int parking_forward = 100;
int parking_backward = 100;

double panel_center_check_point = -0.45;

double parking_dist = 1.1;
double parking_thres = 0.05;

bool front_panel_first_updated_finshed = false;
int first_surface = -1; // -1 : none, 0: front, 1:back, 2:side
bool first_ignore_sequence = false;
int current_waypoint_index = -1;
bool long_panel_first_finshed = false;

double side_center_margin = 1.45;
//-----------------------------------------

int MAX_VEL_SCRIPT = 250;
int MIN_VEL_SCRIPT = 80;

CDriving::CDriving(){

}

CDriving::CDriving(CIMU* _p_imu, CGPS* _p_gps, CLRF* _p_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne, CLMS511* _p_lms511, CSSD* _p_ssd){

    qRegisterMetaType<vector<double>>("vector<double>");
    qRegisterMetaType<mip_filter_linear_acceleration>("mip_filter_linear_acceleration");
    qRegisterMetaType<mip_ahrs_delta_velocity>("mip_ahrs_delta_velocity");

    mpc_imu = _p_imu;
    mpc_gps = _p_gps;
    mpc_drive_lrf = _p_lrf;
    mpc_camera = _p_camera;
    mpc_kinova = _p_kinova;
    mpc_vehicle = _p_vehicle;
    mpc_velodyne = _p_velodyne;
    mpc_lms511 = _p_lms511;
    mpc_ssd = _p_ssd;

    mpc_rgb_d = new CRGBD(_p_lrf);

    connect(mpc_velodyne,SIGNAL(SignalVelodyneParser(bool)),this,SIGNAL(SignalVelodyneParser(bool)));
    connect(mpc_velodyne,SIGNAL(SignalVelodynePanelFound(bool)),this,SLOT(SlotVelodynePanelFound(bool)));
//    connect(mpc_lms511,SIGNAL(SignalLMS511UpdatePoints(vector<vector<double> >)),this,SLOT(SlotLMS511UpdatePoints(vector<vector<double> >)));
    connect(mpc_lms511,SIGNAL(SignalLMS511UpdatePoints(vector<vector<double> >)),this,SIGNAL(SignalLMS511UpdatePoints(vector<vector<double> >)));
    connect(this,SIGNAL(SignalLMS511UpdatePoints(vector<vector<double>>)),mpc_velodyne,SLOT(SlotLMS511UpdatePoints(vector<vector<double> >)));

    connect(mpc_rgb_d,SIGNAL(SignalLRFMapImage(cv::Mat)),this,SIGNAL(SignalLRFMapImage(cv::Mat)));
    connect(mpc_imu,SIGNAL(SignalIMUEuler(vector<double>)),this,SLOT(SlotIMUEuler(vector<double>)));
    connect(mpc_imu,SIGNAL(SignalIMULinearAccel(mip_filter_linear_acceleration)),this,SLOT(SlotIMULinearAccel(mip_filter_linear_acceleration)));
    connect(mpc_imu,SIGNAL(SignalIMUTimestamp(mip_ahrs_internal_timestamp)),this,SLOT(SlotIMUTimestamp(mip_ahrs_internal_timestamp)));
    connect(mpc_imu,SIGNAL(SignalIMUDeltaVelocity(mip_ahrs_delta_velocity)),this,SLOT(SlotIMUDeltaVelocity(mip_ahrs_delta_velocity)));


    connect(this,SIGNAL(SignalVehicleHeading(double)),mpc_vehicle,SLOT(SlotVehicleHeading(double)));

    linear_accel.x = 0;
    linear_accel.y = 0;
    linear_accel.z = 0;

    time_stamp.counts = 0;
    delta_velocity.delta_velocity[0] = 0;
    delta_velocity.delta_velocity[1] = 0;
    delta_velocity.delta_velocity[2] = 0;

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


bool CDriving::IsGPSConnected(){
    return mpc_gps->IsGPSConnected();
}

void CDriving::SetInitGPSpoint()
{
    mpc_gps->SetInitGPS();
//    mpc_gps->CalcGroundGpspoint();
}

void CDriving::SetGroundGPS()
{
}

bool CDriving::CloseGPS()
{
    mpc_gps->GpsClose();
    return true;
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

vector<double> CDriving::GetIMUEuler(){
    return euler_angles;
}

mip_filter_linear_acceleration CDriving::GetIMULinearAccel(){
    return linear_accel;
}

mip_ahrs_internal_timestamp CDriving::GetIMUTimestamp(){
    return time_stamp;
}

mip_ahrs_delta_velocity CDriving::GetIMUDeltaVelocity(){
    return delta_velocity;
}

CVelodyne* CDriving::GetVelodyne(){
    return mpc_velodyne;
}

vector<int> CDriving::GetEncoderValueRaw(){
    vector<int> encoder_value_raw = {0,0};
    if(!mpc_vehicle->IsConnected())
    {
        return encoder_value_raw;
    }
    else
    {
        return current_encoder_value;
    }
}

int CDriving::GetPanelHeadingError(){
    std::vector<double> panel_loc;

    panel_loc = mpc_velodyne->GetPanelCenterLoc();

    double heading_error;

    double error_margin = PI/180.0*(10.0);

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


void CDriving::SetArenaInfo(vector<double> _lb, vector<double> _lt, vector<double> _rt, vector<double> _rb)
{
    m_arena_info.at(0) = _lb;
    m_arena_info.at(1) = _lt;
    m_arena_info.at(2) = _rt;
    m_arena_info.at(3) = _rb;
    mpc_velodyne->SetArenaBoundary(m_arena_info,0,0,0);
}

void CDriving::SetArenaShift(bool _shift_on)
{
    if(_shift_on)
    {
        aligned_initial_heading = euler_angles.at(2);
        if(aligned_initial_heading < 0)
        {
            aligned_initial_heading += 2*PI;
        }
        arena_aligned_compensate = true;
        Sleep(500);
        // Init encoder
        mpc_vehicle->InitEncoder();

    }
    else
    {
        arena_aligned_compensate = false;
    }
}

void CDriving::SetPanelDistance(double _panel_dist_dri, double _panel_dist_par)
{
    side_center_margin = _panel_dist_dri;
    desirable_parking_dist = _panel_dist_par;
}

void CDriving::RefreshArena()
{
    // Aerna info
    m_arena_info = {{0,-45},{-60,-45},{-60,45},{0,45}};
    arena_aligned_compensate = false;
    IMU_update_finished = true;

    // Aligned initial heading
    aligned_initial_heading = 0.0;
    aligned_heading_to_current_differ = 0.0;
    past_encoder_value = {0,0};
    current_encoder_value = {0,0};
    encoder_bias_update = false;
    encoder_bias_1 = 0;
    encoder_bias_2 = 0;
    arena_move_coordinate = {0,0};
    arena_move_coordinate_old = {0,0};
    total_move_distance = 0;



    // parking parameter
    side_center_margin = 0.9;
    desirable_parking_dist = 0.9;

    // ugv info variables
    ugv_heading = 0.0;
    final_parking_heading = -PI;

    panel_slope_norm_x = 0.0;
    panel_slope_norm_y = 0.0;
    panel_way_x = 0.0;
    panel_way_y = 0.0;
    parking_short = true;
    panel_found = true;

    m_dist_error_from_p1_km=0.0;

    euler_angles = {0,0,0};

    mpc_velodyne->SetArenaBoundary(m_arena_info,0,0,0);
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

    if(_fnc_index_ == DRIVE_INX_PARKING_RETRY){
        m_main_fnc_index = DRIVE_INX_PARKING_RETRY;
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
    else if(_fnc_index_ == DRIVE_INX_LOCAL_ON_PANE){
        m_main_fnc_index = DRIVE_INX_LOCAL_ON_PANE;
        this->start();

        return true;
    }
    else if(_fnc_index_ == DRIVE_INX_WRENCH_RECOG){
        m_main_fnc_index = DRIVE_INX_WRENCH_RECOG;
        this->start();

        return true;
    }
    else
        return false;
}

void CDriving::SetDrivingOption(DRIVING_STRUCT _driving_option){

    mtx_driving_struct.lock();
    {
        drive_left = _driving_option.drive_left;
        drive_differ_left = _driving_option.drive_differ_left;
        drive_right = _driving_option.drive_right;
        drive_differ_right = _driving_option.drive_differ_right;

        parking_left = _driving_option.parking_left;
        parking_differ_left = _driving_option.parking_differ_left;
        parking_right = _driving_option.parking_right;
        parking_differ_right = _driving_option.parking_differ_right;
        parking_forward = _driving_option.parking_forward;
        parking_backward = _driving_option.parking_backward;

        panel_center_check_point = _driving_option.panel_center_check_point;

        parking_dist = _driving_option.parking_dist;
        parking_thres = _driving_option.parking_thres;

        side_center_margin = _driving_option.side_center_margin;

        MAX_VEL_SCRIPT = _driving_option.max_vel_script;
        MIN_VEL_SCRIPT = _driving_option.min_vel_script;

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

PARKING_RETRY_STRUCT CDriving::GetParkingRetryOption(){

    PARKING_RETRY_STRUCT parking_struct;

    mxt_parking_retry.lock();
    {
        parking_struct = mstruct_parking_retry;
    }
    mxt_parking_retry.unlock();

    return parking_struct;
}

void CDriving::SetManipulationOption(LRF_VEHICLE_HORIZEN_STRUCT _manipulation_option){

    mxt_lrf_vehicle.lock();
    {
        mstruct_lrf_vehicle = _manipulation_option;
    }
    mxt_lrf_vehicle.unlock();
}

void CDriving::SetManipulationOption(PARKING_RETRY_STRUCT _driving_option){

    mxt_parking_retry.lock();
    {
        mstruct_parking_retry = _driving_option;
    }
    mxt_parking_retry.unlock();
}

void CDriving::SetManipulationOption(WRENCH_RECOGNITION_STRUCT _driving_option){

    mxt_wrench_recog.lock();
    {
        mstruct_wrench_recog = _driving_option;
    }
    mxt_wrench_recog.unlock();
}

WRENCH_RECOGNITION_STRUCT CDriving::GetWrenchRecogOption(){

    WRENCH_RECOGNITION_STRUCT wrench_recog;

    mxt_wrench_recog.lock();
    {
        wrench_recog = mstruct_wrench_recog;
    }
    mxt_wrench_recog.unlock();

    return wrench_recog;
}


void CDriving::SetManipulationOption(VEHICLE_LOCALIZATION_ON_PANEL_STRUCT _driving_option){
    mxt_local_on_panel.lock();
    {
        mstruct_local_on_panel = _driving_option;
    }
    mxt_local_on_panel.unlock();
}

VEHICLE_LOCALIZATION_ON_PANEL_STRUCT CDriving::GetVehicleLocalOnPanelOption(){

    VEHICLE_LOCALIZATION_ON_PANEL_STRUCT local_on_panel;

    mxt_local_on_panel.lock();
    {
        local_on_panel = mstruct_local_on_panel;
    }
    mxt_local_on_panel.unlock();

    return local_on_panel;
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

vector<double> CDriving::GetWaypointError(double _way_x,double _way_y)
{
    vector<double> error_vec;
    double heading_error=0.;
    double dist_error=0.;

    heading_error = PI/2 - atan2(-(_way_x), _way_y);
    if(_way_x > 0)
    {
        dist_error = - sqrt(_way_x * _way_x + _way_y * _way_y);
    }
    else
    {
        dist_error = sqrt(_way_x * _way_x + _way_y * _way_y);
    }

    error_vec.push_back(heading_error);
    error_vec.push_back(dist_error);

    return error_vec;
}

int CDriving::GetParkingControl(vector<double> _waypoint_error)
{
    double heading_error = _waypoint_error.at(0);
    double dist_error = _waypoint_error.at(1);

    int dir = -1;

    if(heading_error < -8.0 / 180.0 * PI) // waypoint on the left
    {
        dir = UGV_move_left;
        return dir;
    }
    else if(heading_error > 8.0 / 180 * PI) // waypoint on the right
    {
        dir = UGV_move_right;
        return dir;
    }
    else if(dist_error < -0.1) // waypoint on the back
    {
        dir = UGV_move_backward;
        return dir;
    }
    else if(dist_error > 0.1) // waypoint on the front
    {
        dir = UGV_move_forward;
        return dir;
    }
    else
    {
        return dir;
    }
}

//----------------------------------------------------------------
// Main Function
//----------------------------------------------------------------

int CDriving::VelGen(double _dist_error)
{
    int velocity=0;
    if(_dist_error >DECEL_START_DIST)
    {
        velocity= (mpc_vehicle->GetVel() + ACCEL_RATE);
        //if(velocity > MAX_VEL)
        if(velocity > MAX_VEL_SCRIPT)
            velocity = MAX_VEL_SCRIPT;
    }
    else
    {
        velocity= (mpc_vehicle->GetVel() + DECEL_RATE);
//        if(velocity < MIN_VEL)
        if(velocity < MIN_VEL_SCRIPT)
            velocity = MIN_VEL_SCRIPT;
    }
    return velocity;
}

int CDriving::VelGen_turn_left()
{
    int velocity=0;
    if(mpc_vehicle->turn_continue_flag_left== true)
    {
        velocity= (mpc_vehicle->GetVel()+ TURN_ACCEL_RATE);
        if(velocity > MAX_VEL_TURN)
            velocity = MAX_VEL_TURN;
    }
    else
    {
        velocity= 100;
        mpc_vehicle->turn_continue_flag_left = true;
    }
    return velocity;
}


int CDriving::VelGen_turn_right()
{
    int velocity=0;
    if(mpc_vehicle->turn_continue_flag_right == true)
    {
        velocity= (mpc_vehicle->GetVel()+ TURN_ACCEL_RATE);
        if(velocity > MAX_VEL_TURN)
            velocity = MAX_VEL_TURN;
    }
    else
    {
        velocity= 100;
        mpc_vehicle->turn_continue_flag_right = true;
    }
    cout << "velocity : " << velocity << endl;

    return velocity;
}

int CDriving::VelGen_parking_turn_left()
{
    int velocity=0;

    if(mpc_vehicle->turn_continue_flag_left == true)
    {
        velocity= (mpc_vehicle->GetVel()+ PARKING_TURN_ACCEL_RATE);
        if(velocity > MAX_VEL_PARKING_TURN)
            velocity = MAX_VEL_PARKING_TURN;
    }
    else
    {
        velocity= 100;
        mpc_vehicle->turn_continue_flag_left = true;
    }

    cout << "velocity : " << velocity << endl;
    return velocity;
}

int CDriving::VelGen_parking_turn_right()
{
    int velocity=0.;

    if(mpc_vehicle->turn_continue_flag_right == true)
    {
        velocity= (mpc_vehicle->GetVel()+ PARKING_TURN_ACCEL_RATE);
        if(velocity > MAX_VEL_PARKING_TURN)
            velocity = MAX_VEL_PARKING_TURN;
    }
    else
    {
        velocity= 100;
        mpc_vehicle->turn_continue_flag_right = true;
    }
    return velocity;
}

int CDriving::VelGen_panel_dist_error(double _dist_error)
{
    int velocity = 0;
    int min_vel = 70;
    if(_dist_error >= 0.8)
    {
        velocity = 90;
    }
    else if(_dist_error < 0.8 && _dist_error >= 0.5)
    {
        velocity = 80;
    }
    else if(_dist_error < 0.5 && _dist_error >= 0.3)
    {
        velocity = 75;
    }
    else if(_dist_error < 0.3)
    {
        velocity = min_vel;
    }

    return velocity;
}


bool CDriving::ParkingRetry(){

    double current_dst = 0;
    PARKING_RETRY_STRUCT parking_option = GetParkingRetryOption();

    current_dst = LrfParkingDistnaceCheck(parking_option.desired_dst);

    while( ((current_dst) > parking_option.max_dst) ||
           ((current_dst) < parking_option.min_dst)
           ){


        current_dst = LrfParkingDistnaceCheck(parking_option.desired_dst);

        double bias = parking_option.desired_dst - current_dst;

        std::cout << "Current Bias : " << bias << std::endl;

        ParkingFrontPanel(bias);
        msleep(300);
    }


    return true;
}

bool CDriving::DriveToPanel(){

    DRIVING_STRUCT driving_struct;

    if(!mpc_vehicle->Connect()){
        std::cout << "Fail to Connection Vehicle" << std::endl;
        return false;
    }

    aligned_initial_heading = euler_angles.at(2);
    if(aligned_initial_heading < 0)
    {
        aligned_initial_heading += 2*PI;
    }
    arena_aligned_compensate = true;

    Sleep(500);
    // Init encoder
    mpc_vehicle->InitEncoder();

    cout << "Driving Start " << endl;
    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_DRIVING);

    if(!InitialSearching(30))
    {
        return false;
    }

    do{

        if(!panel_found)
        {
            driving_struct.direction = UGV_move_forward;
            driving_struct.velocity = 200;
            mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
            Sleep(20);
            continue;
        }

        vector<double> way_point_error = GetWaypointError(mpc_velodyne->GetPanelCenterLoc().at(0) + 1.35,mpc_velodyne->GetPanelCenterLoc().at(1));
        cout << "panel center loc x : " << mpc_velodyne->GetPanelCenterLoc().at(0) << endl;
        cout << "panel center loc y : " << mpc_velodyne->GetPanelCenterLoc().at(1) << endl;

        if(!((mpc_velodyne->GetPanelCenterLoc().at(0) == 0) && (mpc_velodyne->GetPanelCenterLoc().at(0) == 0)))
        {
            if ( (way_point_error.at(1) < 0.5) /*|| (way_point_error.at(0)/way_point_error.at(1) > 1.0) */ )
            {
                mpc_vehicle->Move(UGV_move_backward, 0);
                break;
            }
        }

        switch(GetParkingControl(way_point_error))
        {
        case 3: // turn left
            driving_struct.direction = UGV_move_left;
            driving_struct.velocity = VelGen_turn_left();
            break;
        case 4: // turn right
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity = VelGen_turn_right();
            break;
        case 1: // go forward
            driving_struct.direction = UGV_move_forward;
            driving_struct.velocity =this->VelGen(way_point_error.at(1));
            break;
        case 2: // go backward
            driving_struct.direction = UGV_move_backward;
            driving_struct.velocity =this->VelGen(way_point_error.at(1));
            break;
        default:
            driving_struct.direction = UGV_move_backward;
            driving_struct.velocity =0;
            break;
        }

        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

        Sleep(20);

    }while(true);
    cout << "driving mision done"<< endl;

    mpc_vehicle->Move(UGV_move_backward, 0);

    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_PARKING);

    Sleep(2000);

//    //heading compensate before parking


    // Get panel points
    panel_point_info panel_point_array[6];

    do{
        cout << "Get panelpoints!" << endl;
        driving_struct.direction = UGV_move_backward;
        driving_struct.velocity =100;
        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
        for(int i = 0; i < 6;i++)
        {
            panel_point_array[i].x = mpc_velodyne->GetPCL()->panelpoint_cloud->points[i].x;
            panel_point_array[i].y = mpc_velodyne->GetPCL()->panelpoint_cloud->points[i].y;
        }
        current_waypoint_index = mpc_velodyne->GetCurrentWaypointIndex();
    }while((panel_point_array[0].x == 0) && (panel_point_array[1].x == 0) );

    if((current_waypoint_index == 0) || (current_waypoint_index == 7) || (current_waypoint_index == 6))
    {
        first_ignore_sequence = true;
    }
    cout << " current waypoint index : " << current_waypoint_index << endl;

    driving_struct.direction = UGV_move_backward;
    driving_struct.velocity =0;
    mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    double dist_arr[6] ={0,0,0,0,0,0};
    double min_dist = 1000000.;
    int min_dist_point_index = -1;
    int panel_left_cnt =1;
    int panel_right_cnt =1;


    for(int i=0; i<6; i++)
    {
        dist_arr[i] = 100*(panel_point_array[i].x * panel_point_array[i].x + panel_point_array[i].y * panel_point_array[i].y);

        if(dist_arr[i] < min_dist)
        {
            min_dist = dist_arr[i];
            min_dist_point_index = i;
        }
        if(panel_point_array[i].y < 0 )
            panel_left_cnt = panel_left_cnt + 1;
        else
            panel_right_cnt = panel_right_cnt + 1;
    }

    cout << "index of min distance : " << min_dist_point_index << endl;



    vector<double> rec_imu_data;
    vector<double> rec_imu_data_tmp;
    rec_imu_data = euler_angles;


    double rotate_rad;

    if(panel_left_cnt == 6)
    {
        cout << "all panel left"<< endl;
        if(min_dist_point_index == 5 )
        {
            rotate_rad = atan(abs(panel_point_array[0].y - panel_point_array[min_dist_point_index].y) / abs(panel_point_array[0].x - panel_point_array[min_dist_point_index].x));
        }
        else
        {
            cout << "all panel left and not in index 5"<< endl;
            rotate_rad = atan(abs(panel_point_array[min_dist_point_index+1].y - panel_point_array[min_dist_point_index].y) / abs(panel_point_array[min_dist_point_index+1].x - panel_point_array[min_dist_point_index].x));
        }
    }
    else if(panel_right_cnt == 6)
    {
        cout << "all panel right"<< endl;
        if(min_dist_point_index == 5 )
        {
            rotate_rad = 0.5 * PI - atan(abs(panel_point_array[0].y - panel_point_array[min_dist_point_index].y) / abs(panel_point_array[0].x - panel_point_array[min_dist_point_index].x));
        }
        else
        {
            cout << "all panel right and not in index 5"<< endl;
            rotate_rad = 0.5 * PI - atan(abs(panel_point_array[min_dist_point_index+1].y - panel_point_array[min_dist_point_index].y) / abs(panel_point_array[min_dist_point_index+1].x - panel_point_array[min_dist_point_index].x));
        }
    }
    else
    {
        if(min_dist_point_index == 5)
        {
            cout << "in index 5 on the right"<< endl;
            if( panel_point_array[min_dist_point_index].y < (0 ))
            {
                rotate_rad = atan(abs(panel_point_array[0].y - panel_point_array[min_dist_point_index].y) / abs(panel_point_array[0].x - panel_point_array[min_dist_point_index].x));
            }
            else
            {
                cout << "in index 5 on the left"<< endl;
                rotate_rad = 0.5 * PI - atan(abs(panel_point_array[min_dist_point_index].y - panel_point_array[min_dist_point_index-1].y) / abs(panel_point_array[min_dist_point_index].x - panel_point_array[min_dist_point_index-1].x));
            }
        }
        else
        {
            if( panel_point_array[min_dist_point_index].y < (0 ) )
            {
                cout << "not in index 5 and which is on the right"<< endl;
                rotate_rad = atan(abs(panel_point_array[min_dist_point_index+1].y - panel_point_array[min_dist_point_index].y) / abs(panel_point_array[min_dist_point_index+1].x - panel_point_array[min_dist_point_index].x));
            }

            else
            {
                cout << "not in index 5 and which is on the left"<< endl;
                rotate_rad = 0.5 * PI - atan(abs(panel_point_array[min_dist_point_index].y - panel_point_array[min_dist_point_index-1].y) / abs(panel_point_array[min_dist_point_index].x - panel_point_array[min_dist_point_index-1].x));
            }
        }
    }

    // Get final parking heading value --------------------------------------------------------------------------------------------------------------
    //

    double panel_final_parking_vec_angle = 0;
    do{
        double panel_final_parking_vec_x = panel_point_array[1].x - panel_point_array[0].x;
        double panel_final_parking_vec_y = panel_point_array[1].y - panel_point_array[0].y;
        panel_final_parking_vec_angle = atan2(panel_final_parking_vec_y,panel_final_parking_vec_x);
        if(panel_final_parking_vec_angle < 0)
        {
            panel_final_parking_vec_angle += 2*PI;
        }
    }while(panel_final_parking_vec_angle == 0);

    double ugv_heading_vec_x = -1.0;
    double ugv_heading_vec_y = 0.0;
    double ugv_heading_angle = atan2(ugv_heading_vec_y, ugv_heading_vec_x);
    if(ugv_heading_angle < 0)
    {
        ugv_heading_angle += 2*PI;
    }

    double final_parking_and_ugv_heading_differ = panel_final_parking_vec_angle - PI;

    double current_ugv_heading = (euler_angles).at(2);

    if(current_ugv_heading < 0)
    {
        current_ugv_heading += 2*PI;
    }

    final_parking_heading = current_ugv_heading - final_parking_and_ugv_heading_differ ;

    if(final_parking_heading < 0)
    {
        final_parking_heading += 2*PI;
    }
    else if(final_parking_heading > 2*PI)
    {
        final_parking_heading -= 2*PI;
    }
    //
    // ------------------------------------------------------------------------------------------------------------------------------------------------

    Sleep(1000);

    double frontpanel_init_heading = rec_imu_data.at(2) * 180 / PI;
    double cur_heading_degree_tmp;

    cur_heading_degree_tmp = frontpanel_init_heading;

    while((abs(frontpanel_init_heading + 90 - (rotate_rad * 180 / PI) ) - cur_heading_degree_tmp) > 3)
    {
        cout << " cur_heading prior : " << frontpanel_init_heading << endl;
        cout << " abs(rotate_rad * 180 / pi) : " << abs(rotate_rad * 180 / PI) << endl;
        cout << " cur_heading : " << cur_heading_degree_tmp << endl;

        rec_imu_data_tmp = euler_angles;
        cur_heading_degree_tmp = rec_imu_data_tmp.at(2) * 180 / PI;

//        driving_struct.direction = UGV_move_differ_right;
//        driving_struct.velocity =150;
        driving_struct.direction = UGV_move_right;
        driving_struct.velocity = VelGen_turn_right();
//        driving_struct.velocity = drive_differ_right;
        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

        cout << "turning for compensation" << endl;
        Sleep(20);
    }

    driving_struct.direction = UGV_move_right;
    driving_struct.velocity =0;
    mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

//    driving_struct.direction = UGV_move_backward;
//    driving_struct.velocity =0;

    cout << "finish"<< endl;


    return true;
}

bool CDriving::ParkingFrontPanel(){

    DRIVING_STRUCT driving_struct;

    if(!mpc_vehicle->Connect()){
        std::cout << "Fail to Connection Vehicle" << std::endl;
        return false;
    }

    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_PARKING);


    int _s_deg = 0;
    int _e_deg = 180;

    double panel_length_margin = 0.15;
    double front_center_margin = 1.0;

    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_lrf_index -s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    long* lrf_distance_raw = new long[1081];

    bool front_heading_range_satisfiled = false;

    vector<double> detected_panel_info;

    do{
        if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
            std::cout << "LRF : GetLRFData Error" << std::endl;
        }
        else
        {
            cout << "side center margin : " << side_center_margin << endl;
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            if(!mpc_velodyne->GetLRFPanelFindStatus())
            {
//                driving_struct.direction = UGV_move_differ_left;
//                driving_struct.velocity =parking_differ_left;
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_parking_turn_left();
            }
            else
            {

                do{
                    detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
                }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(0) == 0));

                double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

                if(panel_slope_atan2 < 0)
                {
                    panel_slope_atan2 += 2.0*PI;
                }

                panel_slope_norm_x = -detected_panel_info.at(1);
                panel_slope_norm_y = detected_panel_info.at(0);

                double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if(panel_slope_norm < 0)
                {
                    panel_slope_norm += 2.0*PI;
                }

                double panel_length = detected_panel_info.at(2);

                if((panel_length > 0.5 - panel_length_margin)&&(panel_length < 0.5 + panel_length_margin)) // side
                {
                    if(!front_panel_first_updated_finshed)
                    {
                        first_surface = 2;
                        front_panel_first_updated_finshed = true;
                    }
                    if(long_panel_first_finshed)
                    {
                        first_ignore_sequence = false;
                    }
                    if(first_surface == 0)
                    {
                        first_surface = -1;
                    }

                    double panel_center_to_origin_x = -detected_panel_info.at(3);
                    double panel_center_to_origin_y = -detected_panel_info.at(4);


                    double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                    if(panel_center_to_origin_slope < 0)
                    {
                        panel_center_to_origin_slope += 2.0*PI;
                    }

                    double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                    if (norm_to_origin_angle > PI)
                    {
                        norm_to_origin_angle -= PI;
                    }

                    if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                    {
                        panel_slope_norm_x = -panel_slope_norm_x;
                        panel_slope_norm_y = -panel_slope_norm_y;
                    }

                    double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                    if (panel_slope_norm_angle < 0)
                    {
                        panel_slope_norm_angle += 2*PI;
                    }

                    //// Test section ----------------////////////////

                    double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                    double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                    double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                    if(angle_between_norm_and_panel_center_to_origin > PI)
                    {
                        angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                    }

                    double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                    double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                    panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                    panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);

                    double panel_center_to_way_x = panel_way_x - detected_panel_info.at(3);
                    double panel_center_to_way_y = panel_way_y - detected_panel_info.at(4);

                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_center_to_way_y - panel_center_to_origin_y*panel_center_to_way_x;

                    //// ------------------------------////////////////

                    if(outer_product_panel_center_norm  > 0)
                    {
                        if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_left;
                            driving_struct.velocity = parking_differ_left;
//                            driving_struct.direction = UGV_move_left;
//                            driving_struct.velocity = VelGen_parking_turn_left();
                        }
                        else if ( ((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 2*PI)))
                        {
                            driving_struct.direction = UGV_move_differ_right;
                            driving_struct.velocity = parking_differ_right;
//                            driving_struct.direction = UGV_move_right;
//                            driving_struct.velocity = VelGen_parking_turn_right();
                        }
                        else
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity = parking_forward;
                        }
                    }
                    else
                    {
                        if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_left;
                            driving_struct.velocity = parking_differ_left;
//                            driving_struct.direction = UGV_move_left;
//                            driving_struct.velocity = VelGen_parking_turn_left();
                        }
                        else if ((panel_slope_norm_angle > (PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_right;
                            driving_struct.velocity = parking_differ_right;
//                            driving_struct.direction = UGV_move_right;
//                            driving_struct.velocity = VelGen_parking_turn_right();
                        }
                        else
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity = parking_forward;
                        }
                    }
                }
                else if(panel_length > 2.0)
                {
                    driving_struct.direction = UGV_move_differ_left;
                    driving_struct.velocity =parking_differ_left;
//                    driving_struct.direction = UGV_move_left;
//                    driving_struct.velocity = VelGen_parking_turn_left();

                }
                else// front or back
                {

                    long_panel_first_finshed = true;

                    double current_ugv_heading = (euler_angles).at(2);

                    if(current_ugv_heading < 0)
                    {
                        current_ugv_heading += 2*PI;
                    }

                    double final_parking_heading_min = (final_parking_heading - 0.8);
                    double final_parking_heading_max = (final_parking_heading + 0.8);

                    if (final_parking_heading_min < 0)
                    {
                        final_parking_heading_min += 2*PI;
                    }
                    else if (final_parking_heading_min > 2*PI)
                    {
                        final_parking_heading_min -= 2*PI;
                    }

                    if (final_parking_heading_max < 0)
                    {
                        final_parking_heading_max += 2*PI;
                    }
                    else if (final_parking_heading_max > 2*PI)
                    {
                        final_parking_heading_max -= 2*PI;
                    }


                    if(final_parking_heading_max > final_parking_heading_min)
                    {
                        if( (current_ugv_heading > final_parking_heading_min ) && (current_ugv_heading < final_parking_heading_max))
                        {

                            if(!first_ignore_sequence)
                            {
                                front_heading_range_satisfiled = true;
                                front_center_margin = 0.8;
                            }
                            else
                            {
                                front_heading_range_satisfiled = false;
                                front_center_margin = 1.2;
                            }

                        }
                        else
                        {
                            if(!front_panel_first_updated_finshed)
                            {
                                first_surface = 1;
                                front_panel_first_updated_finshed = true;
                            }

                            front_heading_range_satisfiled = false;
                            front_center_margin = 1.2;
                        }
                    }
                    else
                    {
                        if( ((current_ugv_heading > final_parking_heading_min ) && (current_ugv_heading < 2*PI )) ||  ((current_ugv_heading < final_parking_heading_max) && (current_ugv_heading >= 0)))
                        {
//                            front_heading_range_satisfiled = true;
//                            front_center_margin = 0.8;
                            if(!first_ignore_sequence)
                            {
                                front_heading_range_satisfiled = true;
                                front_center_margin = 0.8;
                            }
                            else
                            {
                                front_heading_range_satisfiled = false;
                                front_center_margin = 1.2;
                            }
                        }
                        else
                        {
//                            front_heading_range_satisfiled = false;
//                            front_center_margin = 1.0;
                            if(!front_panel_first_updated_finshed)
                            {
                                first_surface = 1;
                                front_panel_first_updated_finshed = true;
                            }

                            front_heading_range_satisfiled = false;
                            front_center_margin = 1.2;
                        }
                    }

                    double panel_center_to_origin_x = -detected_panel_info.at(3);
                    double panel_center_to_origin_y = -detected_panel_info.at(4);


                    double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                    if(panel_center_to_origin_slope < 0)
                    {
                        panel_center_to_origin_slope += 2.0*PI;
                    }

                    double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                    if (norm_to_origin_angle > PI)
                    {
                        norm_to_origin_angle -= PI;
                    }

                    if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                    {
                        panel_slope_norm_x = -panel_slope_norm_x;
                        panel_slope_norm_y = -panel_slope_norm_y;
                    }

                    double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                    if (panel_slope_norm_angle < 0)
                    {
                        panel_slope_norm_angle += 2*PI;
                    }


                    //// Test section ----------------////////////////

                    double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                    double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                    double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                    if(angle_between_norm_and_panel_center_to_origin > PI)
                    {
                        angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                    }

                    double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                    double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                    panel_way_x = panel_way_anchor_x + front_center_margin*(-panel_slope_norm_y_norm);
                    panel_way_y = panel_way_anchor_y + front_center_margin*(panel_slope_norm_x_norm);

                    double panel_center_to_way_x = panel_way_x - detected_panel_info.at(3);
                    double panel_center_to_way_y = panel_way_y - detected_panel_info.at(4);

                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_center_to_way_y - panel_center_to_origin_y*panel_center_to_way_x;

                    //// ------------------------------////////////////

                    double front_panel_center_to_origin_x = - (detected_panel_info.at(3) + panel_center_check_point);
                    double front_panel_center_to_origin_y = - detected_panel_info.at(4);
                    double front_panel_center_to_origin_angle = atan2(front_panel_center_to_origin_y, front_panel_center_to_origin_x);

                    if (front_panel_center_to_origin_angle < 0)
                    {
                        front_panel_center_to_origin_angle += 2*PI;
                    }

                    double angle_between_norm_origin = front_panel_center_to_origin_angle - panel_slope_norm_angle;

                    if( abs(angle_between_norm_origin) > PI)
                    {
                        angle_between_norm_origin = 2*PI - abs(angle_between_norm_origin);
                    }

                    double front_panel_center_x_to_ugv = sqrt((detected_panel_info.at(3) + panel_center_check_point)*(detected_panel_info.at(3) + panel_center_check_point) + detected_panel_info.at(4)*detected_panel_info.at(4)) * sin(angle_between_norm_origin);


                    double line_distance_to_origin = 0;

                    if (detected_panel_info.at(0) == 0)
                    {
                        line_distance_to_origin = detected_panel_info.at(3);
                    }
                    else
                    {
                        double line_eq_a = detected_panel_info.at(1) / detected_panel_info.at(0);
                        double line_eq_b = detected_panel_info.at(4) - line_eq_a*detected_panel_info.at(3);

                        line_distance_to_origin = abs(line_eq_b)/sqrt(line_eq_a*line_eq_a*+1);
                    }

//                    if ( (line_distance_to_origin < (side_center_margin + 0.25)) && (abs(panel_slope) > (75.0/180.0*PI)))
//                    {
//                        driving_struct.direction = UGV_move_forward;
//                        driving_struct.velocity = parking_forward;
//                    }

//                    else
//                    {

                    if(front_heading_range_satisfiled == true)
                    {
                        double outer_product_check = front_panel_center_to_origin_x * panel_slope_norm_y - front_panel_center_to_origin_y * panel_slope_norm_x;
                        if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_left;
                            driving_struct.velocity = parking_differ_left;
//                            driving_struct.direction = UGV_move_left;
//                            driving_struct.velocity = VelGen_parking_turn_left();
                        }
                        else if (((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 2*PI)))
                        {
                            driving_struct.direction = UGV_move_differ_right;
                            driving_struct.velocity = parking_differ_right;
//                            driving_struct.direction = UGV_move_right;
//                            driving_struct.velocity = VelGen_parking_turn_right();
                        }
                        else
                        {
                            if(!front_panel_first_updated_finshed)
                            {
                                first_surface = 0;
                                front_panel_first_updated_finshed = true;
                            }

                            if(abs(front_panel_center_x_to_ugv) < 0.1)
                            {
                                break;
                            }
                            else
                            {
                                if (outer_product_check > 0)
                                {
                                    driving_struct.direction = UGV_move_forward;
                                    driving_struct.velocity =70;
                                }
                                else
                                {
                                    driving_struct.direction = UGV_move_backward;
                                    driving_struct.velocity =70;
                                }
                            }
                        }

                    }
                    else
                    {
                        if(outer_product_panel_center_norm  > 0)
                        {

                            if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                            {
                                driving_struct.direction = UGV_move_differ_left;
                                driving_struct.velocity = parking_differ_left;
//                                driving_struct.direction = UGV_move_left;
//                                driving_struct.velocity = VelGen_parking_turn_left();
                            }
                            else if (((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 2*PI)))
                            {
                                driving_struct.direction = UGV_move_differ_right;
                                driving_struct.velocity = parking_differ_right;
//                                driving_struct.direction = UGV_move_right;
//                                driving_struct.velocity = VelGen_parking_turn_right();
                            }
                            else
                            {
                                if(!front_panel_first_updated_finshed)
                                {
                                    first_surface = 1;
                                    front_panel_first_updated_finshed = true;
                                }

                                driving_struct.direction = UGV_move_forward;
                                driving_struct.velocity = parking_forward;
                            }
                        }
                        else
                        {
                            if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI - (5.0/180.0*PI))))
                            {
                                driving_struct.direction = UGV_move_differ_left;
                                driving_struct.velocity = parking_differ_left;
//                                driving_struct.direction = UGV_move_left;
//                                driving_struct.velocity = VelGen_parking_turn_left();
                            }
                            else if ((panel_slope_norm_angle > (PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                            {
                                driving_struct.direction = UGV_move_differ_right;
                                driving_struct.velocity = parking_differ_right;
//                                driving_struct.direction = UGV_move_right;
//                                driving_struct.velocity = VelGen_parking_turn_right();
                            }
                            else
                            {
                                driving_struct.direction = UGV_move_forward;
                                driving_struct.velocity = parking_forward;
                            }
                        }
                    }

//                    }
                }

            }

            cout << "driving direction : " << driving_struct.direction << endl;

            mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
        }
    }while(driving_struct.driving_mission);

    Sleep(1000);

    // Control the distance between front panel and ugv-----------------------------------------------
    //
    //

    bool control_panel_front_dist = false;
    if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
        std::cout << "LRF : GetLRFData Error" << std::endl;
    }
    else
    {
        memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
        mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

        if(!mpc_velodyne->GetLRFPanelFindStatus())
        {
//            driving_struct.direction = UGV_move_differ_left;
//            driving_struct.velocity = parking_differ_left;
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity = VelGen_parking_turn_right();
        }
        else
        {

            do{
                detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
            }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(1) == 0));

            double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

            if(panel_slope_atan2 < 0)
            {
                panel_slope_atan2 += 2.0*PI;
            }

            panel_slope_norm_x = -detected_panel_info.at(1);
            panel_slope_norm_y = detected_panel_info.at(0);


            double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

            if(panel_slope_norm < 0)
            {
                panel_slope_norm += 2.0*PI;
            }

            double panel_length = detected_panel_info.at(2);

            if(panel_length > 0.10) // any panel
            {
                double panel_center_to_origin_x = -detected_panel_info.at(3);
                double panel_center_to_origin_y = -detected_panel_info.at(4);

                double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                if(panel_center_to_origin_slope < 0)
                {
                    panel_center_to_origin_slope += 2.0*PI;
                }

                double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                if (norm_to_origin_angle > PI)
                {
                    norm_to_origin_angle -= PI;
                }

                if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                {
                    panel_slope_norm_x = -panel_slope_norm_x;
                    panel_slope_norm_y = -panel_slope_norm_y;
                }

                double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if (panel_slope_norm_angle < 0)
                {
                    panel_slope_norm_angle += 2*PI;
                }

                //// Test section ----------------////////////////

                double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                if(angle_between_norm_and_panel_center_to_origin > PI)
                {
                    angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                }

                double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);

                double panel_center_to_way_x = panel_way_x - detected_panel_info.at(3);
                double panel_center_to_way_y = panel_way_y - detected_panel_info.at(4);

                double outer_product_panel_center_norm = panel_center_to_origin_x*panel_center_to_way_y - panel_center_to_origin_y*panel_center_to_way_x;

                //// ------------------------------////////////////
                double dist_to_ugv_center = 0;

                if(detected_panel_info.at(0) == 0)
                {
                    dist_to_ugv_center = detected_panel_info.at(3);
                }
                else
                {
                    double lrf_eq_a = detected_panel_info.at(1)/detected_panel_info.at(0);
                    double lrf_eq_b = detected_panel_info.at(4) - lrf_eq_a*detected_panel_info.at(3);

                    double dist_target_x = 0.4;
                    double dist_target_y = 0;

                    dist_to_ugv_center = abs(-lrf_eq_a*dist_target_x + dist_target_y - lrf_eq_b)/sqrt(lrf_eq_a*lrf_eq_a+1);
                }

                double dist_margin = 0.1;
                cout << "distance : " << dist_to_ugv_center << endl;
                if(abs(dist_to_ugv_center - 1.05) > dist_margin)
                {
//                    PanelFrontDistanceControlByLMS511();
                    control_panel_front_dist = true;
                }
                else
                {
                    control_panel_front_dist = false;
                }
            }
        }

    }

    if(control_panel_front_dist)
    {
//        ParkingDistanceControl();
    }

//    DrivieByOdometer(euler_angles.at(2),0.5);

    Sleep(2000);

    //
    //
    // -----------------------------------------------------------------------------------------------
    cout << "Parking finished !" << endl;

//    mpc_velodyne->SetVelodyneThread(false);
    return true;
}

bool CDriving::ParkingFrontPanel(double _bias){

    first_ignore_sequence = true;
    long_panel_first_finshed = false;

    side_center_margin += _bias;


    DRIVING_STRUCT driving_struct;

    if(!mpc_vehicle->Connect()){
        std::cout << "Fail to Connection Vehicle" << std::endl;
        return false;
    }

    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_PARKING);


    int _s_deg = 0;
    int _e_deg = 180;

    double panel_length_margin = 0.15;
    double front_center_margin = 1.0;

    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_lrf_index -s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    long* lrf_distance_raw = new long[1081];

    bool front_heading_range_satisfiled = false;

    vector<double> detected_panel_info;

    do{
        if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
            std::cout << "LRF : GetLRFData Error" << std::endl;
        }
        else
        {

            cout << "%% side center margin : " << side_center_margin << endl;
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            if(!mpc_velodyne->GetLRFPanelFindStatus())
            {
//                driving_struct.direction = UGV_move_differ_left;
//                driving_struct.velocity =parking_differ_left;
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_parking_turn_left();
            }
            else
            {

                do{
                    detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
                }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(0) == 0));

                double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

                if(panel_slope_atan2 < 0)
                {
                    panel_slope_atan2 += 2.0*PI;
                }

                panel_slope_norm_x = -detected_panel_info.at(1);
                panel_slope_norm_y = detected_panel_info.at(0);

                double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if(panel_slope_norm < 0)
                {
                    panel_slope_norm += 2.0*PI;
                }

                double panel_length = detected_panel_info.at(2);

                if((panel_length > 0.5 - panel_length_margin)&&(panel_length < 0.5 + panel_length_margin)) // side
                {
                    if(!front_panel_first_updated_finshed)
                    {
                        first_surface = 2;
                        front_panel_first_updated_finshed = true;
                    }
                    if(long_panel_first_finshed)
                    {
                        first_ignore_sequence = false;
                    }
                    if(first_surface == 0)
                    {
                        first_surface = -1;
                    }

                    double panel_center_to_origin_x = -detected_panel_info.at(3);
                    double panel_center_to_origin_y = -detected_panel_info.at(4);


                    double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                    if(panel_center_to_origin_slope < 0)
                    {
                        panel_center_to_origin_slope += 2.0*PI;
                    }

                    double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                    if (norm_to_origin_angle > PI)
                    {
                        norm_to_origin_angle -= PI;
                    }

                    if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                    {
                        panel_slope_norm_x = -panel_slope_norm_x;
                        panel_slope_norm_y = -panel_slope_norm_y;
                    }

                    double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                    if (panel_slope_norm_angle < 0)
                    {
                        panel_slope_norm_angle += 2*PI;
                    }

                    //// Test section ----------------////////////////

                    double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                    double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                    double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                    if(angle_between_norm_and_panel_center_to_origin > PI)
                    {
                        angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                    }

                    double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                    double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                    panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                    panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);

                    double panel_center_to_way_x = panel_way_x - detected_panel_info.at(3);
                    double panel_center_to_way_y = panel_way_y - detected_panel_info.at(4);

                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_center_to_way_y - panel_center_to_origin_y*panel_center_to_way_x;

                    //// ------------------------------////////////////

                    if(outer_product_panel_center_norm  > 0)
                    {
                        if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_left;
                            driving_struct.velocity = parking_left;
//                            driving_struct.direction = UGV_move_left;
//                            driving_struct.velocity = VelGen_parking_turn_left();
                        }
                        else if ( ((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 2*PI)))
                        {
                            driving_struct.direction = UGV_move_differ_right;
                            driving_struct.velocity = parking_differ_right;
//                            driving_struct.direction = UGV_move_right;
//                            driving_struct.velocity = VelGen_parking_turn_right();
                        }
                        else
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity = parking_forward;
                        }
                    }
                    else
                    {
                        if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_left;
                            driving_struct.velocity = parking_differ_left;
//                            driving_struct.direction = UGV_move_left;
//                            driving_struct.velocity = VelGen_parking_turn_left();
                        }
                        else if ((panel_slope_norm_angle > (PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_right;
                            driving_struct.velocity = parking_right;
//                            driving_struct.direction = UGV_move_right;
//                            driving_struct.velocity = VelGen_parking_turn_right();
                        }
                        else
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity = parking_forward;
                        }
                    }
                }
                else if(panel_length > 2.0)
                {
                    driving_struct.direction = UGV_move_differ_left;
                    driving_struct.velocity =parking_differ_left;
//                    driving_struct.direction = UGV_move_left;
//                    driving_struct.velocity = VelGen_parking_turn_left();

                }
                else// front or back
                {

                    long_panel_first_finshed = true;

                    double current_ugv_heading = (euler_angles).at(2);

                    if(current_ugv_heading < 0)
                    {
                        current_ugv_heading += 2*PI;
                    }

                    double final_parking_heading_min = (final_parking_heading - 0.8);
                    double final_parking_heading_max = (final_parking_heading + 0.8);

                    if (final_parking_heading_min < 0)
                    {
                        final_parking_heading_min += 2*PI;
                    }
                    else if (final_parking_heading_min > 2*PI)
                    {
                        final_parking_heading_min -= 2*PI;
                    }

                    if (final_parking_heading_max < 0)
                    {
                        final_parking_heading_max += 2*PI;
                    }
                    else if (final_parking_heading_max > 2*PI)
                    {
                        final_parking_heading_max -= 2*PI;
                    }


                    if(final_parking_heading_max > final_parking_heading_min)
                    {
                        if( (current_ugv_heading > final_parking_heading_min ) && (current_ugv_heading < final_parking_heading_max))
                        {

                            if(!first_ignore_sequence)
                            {
                                front_heading_range_satisfiled = true;
                                front_center_margin = 0.8;
                            }
                            else
                            {
                                front_heading_range_satisfiled = false;
                                front_center_margin = 1.2;
                            }

                        }
                        else
                        {
                            if(!front_panel_first_updated_finshed)
                            {
                                first_surface = 1;
                                front_panel_first_updated_finshed = true;
                            }

                            front_heading_range_satisfiled = false;
                            front_center_margin = 1.2;
                        }
                    }
                    else
                    {
                        if( ((current_ugv_heading > final_parking_heading_min ) && (current_ugv_heading < 2*PI )) ||  ((current_ugv_heading < final_parking_heading_max) && (current_ugv_heading >= 0)))
                        {
//                            front_heading_range_satisfiled = true;
//                            front_center_margin = 0.8;
                            if(!first_ignore_sequence)
                            {
                                front_heading_range_satisfiled = true;
                                front_center_margin = 0.8;
                            }
                            else
                            {
                                front_heading_range_satisfiled = false;
                                front_center_margin = 1.2;
                            }
                        }
                        else
                        {
//                            front_heading_range_satisfiled = false;
//                            front_center_margin = 1.0;
                            if(!front_panel_first_updated_finshed)
                            {
                                first_surface = 1;
                                front_panel_first_updated_finshed = true;
                            }

                            front_heading_range_satisfiled = false;
                            front_center_margin = 1.2;
                        }
                    }

                    double panel_center_to_origin_x = -detected_panel_info.at(3);
                    double panel_center_to_origin_y = -detected_panel_info.at(4);


                    double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                    if(panel_center_to_origin_slope < 0)
                    {
                        panel_center_to_origin_slope += 2.0*PI;
                    }

                    double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                    if (norm_to_origin_angle > PI)
                    {
                        norm_to_origin_angle -= PI;
                    }

                    if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                    {
                        panel_slope_norm_x = -panel_slope_norm_x;
                        panel_slope_norm_y = -panel_slope_norm_y;
                    }

                    double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                    if (panel_slope_norm_angle < 0)
                    {
                        panel_slope_norm_angle += 2*PI;
                    }


                    //// Test section ----------------////////////////

                    double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                    double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                    double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                    if(angle_between_norm_and_panel_center_to_origin > PI)
                    {
                        angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                    }

                    double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                    double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                    panel_way_x = panel_way_anchor_x + front_center_margin*(-panel_slope_norm_y_norm);
                    panel_way_y = panel_way_anchor_y + front_center_margin*(panel_slope_norm_x_norm);

                    double panel_center_to_way_x = panel_way_x - detected_panel_info.at(3);
                    double panel_center_to_way_y = panel_way_y - detected_panel_info.at(4);

                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_center_to_way_y - panel_center_to_origin_y*panel_center_to_way_x;

                    //// ------------------------------////////////////

                    double front_panel_center_to_origin_x = - (detected_panel_info.at(3) + panel_center_check_point);
                    double front_panel_center_to_origin_y = - detected_panel_info.at(4);
                    double front_panel_center_to_origin_angle = atan2(front_panel_center_to_origin_y, front_panel_center_to_origin_x);

                    if (front_panel_center_to_origin_angle < 0)
                    {
                        front_panel_center_to_origin_angle += 2*PI;
                    }

                    double angle_between_norm_origin = front_panel_center_to_origin_angle - panel_slope_norm_angle;

                    if( abs(angle_between_norm_origin) > PI)
                    {
                        angle_between_norm_origin = 2*PI - abs(angle_between_norm_origin);
                    }

                    double front_panel_center_x_to_ugv = sqrt((detected_panel_info.at(3) + panel_center_check_point)*(detected_panel_info.at(3) + panel_center_check_point) + detected_panel_info.at(4)*detected_panel_info.at(4)) * sin(angle_between_norm_origin);

                    double line_distance_to_origin = 0;

                    if (detected_panel_info.at(0) == 0)
                    {
                        line_distance_to_origin = detected_panel_info.at(3);
                    }
                    else
                    {
                        double line_eq_a = detected_panel_info.at(1) / detected_panel_info.at(0);
                        double line_eq_b = detected_panel_info.at(4) - line_eq_a*detected_panel_info.at(3);

                        line_distance_to_origin = abs(line_eq_b)/sqrt(line_eq_a*line_eq_a*+1);
                    }

//                    if ( (line_distance_to_origin < (side_center_margin + 0.25)) && (abs(panel_slope) > (75.0/180.0*PI)))
//                    {
//                        driving_struct.direction = UGV_move_forward;
//                        driving_struct.velocity = parking_forward;
//                    }

//                    else
//                    {

                    if(front_heading_range_satisfiled == true)
                    {
                        double outer_product_check = front_panel_center_to_origin_x * panel_slope_norm_y - front_panel_center_to_origin_y * panel_slope_norm_x;
                        if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_left;
                            driving_struct.velocity = parking_differ_left;
//                            driving_struct.direction = UGV_move_left;
//                            driving_struct.velocity = VelGen_parking_turn_left();
                        }
                        else if (((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 2*PI)))
                        {
                            driving_struct.direction = UGV_move_differ_right;
                            driving_struct.velocity = parking_differ_right;
//                            driving_struct.direction = UGV_move_right;
//                            driving_struct.velocity = VelGen_parking_turn_right();
                        }
                        else
                        {
                            if(!front_panel_first_updated_finshed)
                            {
                                first_surface = 0;
                                front_panel_first_updated_finshed = true;
                            }

                            if(abs(front_panel_center_x_to_ugv) < 0.1)
                            {
                                break;
                            }
                            else
                            {
                                if (outer_product_check > 0)
                                {
                                    driving_struct.direction = UGV_move_forward;
                                    driving_struct.velocity =70;
                                }
                                else
                                {
                                    driving_struct.direction = UGV_move_backward;
                                    driving_struct.velocity =70;
                                }
                            }
                        }

                    }
                    else
                    {
                        if(outer_product_panel_center_norm  > 0)
                        {

                            if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                            {
                                driving_struct.direction = UGV_move_differ_left;
                                driving_struct.velocity = parking_differ_left;
//                                driving_struct.direction = UGV_move_left;
//                                driving_struct.velocity = VelGen_parking_turn_left();
                            }
                            else if (((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 2*PI)))
                            {
                                driving_struct.direction = UGV_move_differ_right;
                                driving_struct.velocity = parking_differ_right;
//                                driving_struct.direction = UGV_move_right;
//                                driving_struct.velocity = VelGen_parking_turn_right();
                            }
                            else
                            {
                                if(!front_panel_first_updated_finshed)
                                {
                                    first_surface = 1;
                                    front_panel_first_updated_finshed = true;
                                }

                                driving_struct.direction = UGV_move_forward;
                                driving_struct.velocity = parking_forward;
                            }
                        }
                        else
                        {
                            if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI - (5.0/180.0*PI))))
                            {
                                driving_struct.direction = UGV_move_differ_left;
                                driving_struct.velocity = parking_differ_left;
//                                driving_struct.direction = UGV_move_left;
//                                driving_struct.velocity = VelGen_parking_turn_left();
                            }
                            else if ((panel_slope_norm_angle > (PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                            {
                                driving_struct.direction = UGV_move_differ_right;
                                driving_struct.velocity = parking_differ_right;
//                                driving_struct.direction = UGV_move_right;
//                                driving_struct.velocity = VelGen_parking_turn_right();
                            }
                            else
                            {
                                driving_struct.direction = UGV_move_forward;
                                driving_struct.velocity = parking_forward;
                            }
                        }
                    }

//                    }
                }

            }

            cout << "driving direction : " << driving_struct.direction << endl;

            mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
        }
    }while(driving_struct.driving_mission);

    Sleep(1000);

    // Control the distance between front panel and ugv-----------------------------------------------
    //
    //

    bool control_panel_front_dist = false;
    if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
        std::cout << "LRF : GetLRFData Error" << std::endl;
    }
    else
    {
        memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
        mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

        if(!mpc_velodyne->GetLRFPanelFindStatus())
        {
//            driving_struct.direction = UGV_move_differ_left;
//            driving_struct.velocity = parking_differ_left;
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity = VelGen_parking_turn_right();
        }
        else
        {

            do{
                detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
            }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(1) == 0));

            double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

            if(panel_slope_atan2 < 0)
            {
                panel_slope_atan2 += 2.0*PI;
            }

            panel_slope_norm_x = -detected_panel_info.at(1);
            panel_slope_norm_y = detected_panel_info.at(0);


            double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

            if(panel_slope_norm < 0)
            {
                panel_slope_norm += 2.0*PI;
            }

            double panel_length = detected_panel_info.at(2);

            if(panel_length > 0.10) // any panel
            {
                double panel_center_to_origin_x = -detected_panel_info.at(3);
                double panel_center_to_origin_y = -detected_panel_info.at(4);

                double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                if(panel_center_to_origin_slope < 0)
                {
                    panel_center_to_origin_slope += 2.0*PI;
                }

                double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                if (norm_to_origin_angle > PI)
                {
                    norm_to_origin_angle -= PI;
                }

                if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                {
                    panel_slope_norm_x = -panel_slope_norm_x;
                    panel_slope_norm_y = -panel_slope_norm_y;
                }

                double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if (panel_slope_norm_angle < 0)
                {
                    panel_slope_norm_angle += 2*PI;
                }

                //// Test section ----------------////////////////

                double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                if(angle_between_norm_and_panel_center_to_origin > PI)
                {
                    angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                }

                double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);

                double panel_center_to_way_x = panel_way_x - detected_panel_info.at(3);
                double panel_center_to_way_y = panel_way_y - detected_panel_info.at(4);

                double outer_product_panel_center_norm = panel_center_to_origin_x*panel_center_to_way_y - panel_center_to_origin_y*panel_center_to_way_x;

                //// ------------------------------////////////////
                double dist_to_ugv_center = 0;

                if(detected_panel_info.at(0) == 0)
                {
                    dist_to_ugv_center = detected_panel_info.at(3);
                }
                else
                {
                    double lrf_eq_a = detected_panel_info.at(1)/detected_panel_info.at(0);
                    double lrf_eq_b = detected_panel_info.at(4) - lrf_eq_a*detected_panel_info.at(3);

                    double dist_target_x = 0.4;
                    double dist_target_y = 0;

                    dist_to_ugv_center = abs(-lrf_eq_a*dist_target_x + dist_target_y - lrf_eq_b)/sqrt(lrf_eq_a*lrf_eq_a+1);
                }

                double dist_margin = 0.1;
                if(abs(dist_to_ugv_center - 1.05) > dist_margin)
                {
//                    PanelFrontDistanceControlByLMS511();
                    control_panel_front_dist = true;
                }
                else
                {
                    control_panel_front_dist = false;
                }
            }
        }

    }

    if(control_panel_front_dist)
    {
//        ParkingDistanceControl();
    }

//    DrivieByOdometer(euler_angles.at(2),0.5);

    Sleep(2000);

    //
    //
    // -----------------------------------------------------------------------------------------------
    cout << "Parking finished !" << endl;

//    mpc_velodyne->SetVelodyneThread(false);
    return true;
}

double CDriving::LrfParkingDistnaceCheck(double _desirable_dist)
{
    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_PARKING);


    int _s_deg = 0;
    int _e_deg = 180;

    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_lrf_index -s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    long* lrf_distance_raw = new long[1081];


    vector<double> detected_panel_info;

    do{
    if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
        std::cout << "LRF : GetLRFData Error" << std::endl;
    }
    else
    {
        cout << "side center margin : " << side_center_margin << endl;
        memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
        mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

        detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
    }
    }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(0) == 0));

//    double current_dist = abs(detected_panel_info.at(4));


    double current_dist = mpc_velodyne->GetCurrentUGVDepth();


//    double return_bias = _desirable_dist - current_dist;

    cout << "%% current dist : " << current_dist << endl;

    return current_dist;
}

double CDriving::DriveByVelodyne(double _desirable_pos)
{
    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_PARKING);
    Sleep(300);

    DRIVING_STRUCT driving_struct;

    double current_ugv_pos = 0;

    int empty_count = 0;
    do{
        do{
            current_ugv_pos = mpc_velodyne->GetCurrentUGVPosition();
        }while(current_ugv_pos == 0);

        cout << " velodyne point number : " << mpc_velodyne->GetPCL()->velo_panel_cloud->points.size() << endl;
        cout << " current ugv pos : " << current_ugv_pos << endl;

        if(mpc_velodyne->GetPCL()->velo_panel_cloud->points.size() == 0)
        {
            empty_count++;
        }
        else
        {
            empty_count = 0;
        }

        if(empty_count > 10)
        {
            cout << "fuck" << endl;

            break;
        }

        if(current_ugv_pos < _desirable_pos)
        {
            driving_struct.direction = UGV_move_forward;
        }
        else
        {
            driving_struct.direction = UGV_move_backward;
        }

        driving_struct.velocity = VelGen_panel_dist_error(abs(_desirable_pos - current_ugv_pos));

        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

        Sleep(10);
    }while(abs(current_ugv_pos - _desirable_pos) > 0.05);

    driving_struct.velocity = 0;
    mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    Sleep(2000);

    current_ugv_pos = mpc_velodyne->GetCurrentUGVPosition();
    double control_err = _desirable_pos - current_ugv_pos;

    if(current_ugv_pos == -1)
    {
        control_err = 0;
    }

    return control_err;
}


double CDriving::DriveByRenchRecog(int _rench_index, double _dist_to_panel)
{

    DRIVING_STRUCT driving_struct;

    vector<int> rench_recog_list;

    double rench_desirable_differ_current = _rench_index;

    int max_rench_cnt = 0;
    long vel_up_cnt = 0;
    bool new_rench_update = false;
    do{

        rench_recog_list = mpc_ssd->GetRenchLocList();
        if (max_rench_cnt < (int)rench_recog_list.size())
        {
            new_rench_update = true;
            vel_up_cnt = 0;
            max_rench_cnt = (int)rench_recog_list.size();
        }
        else if((int)rench_recog_list.size() <= max_rench_cnt)
        {
            new_rench_update = false;
            vel_up_cnt++;
        }

        if(rench_recog_list.size() == 0)
        {
            driving_struct.direction = UGV_move_forward;
//            driving_struct.velocity = 45;
            driving_struct.velocity = VelGen_Cnt(vel_up_cnt,new_rench_update,500);
            mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
//            Sleep(3);
            continue;
        }



        bool recog_err = false;
        if(rench_recog_list.size() >= 2)
        {
            for(int rench_index = 0; rench_index < (rench_recog_list.size()-1);rench_index++)
            {
                double rench_distance = rench_recog_list.at(rench_index + 1) - rench_recog_list.at(rench_index);
                if( (rench_distance > 250) ||  (rench_distance < 50))
                {
                    recog_err = true;
                }
            }
        }


        if (recog_err)
            continue;


        rench_desirable_differ_current = (int)_rench_index - (int)rench_recog_list.size();

        if(rench_desirable_differ_current > 0)
        {
            driving_struct.direction = UGV_move_forward;
        }
        else
        {
            driving_struct.direction = UGV_move_backward;
        }

//        if(vel_tick % 10 == 0)
//            driving_struct.velocity = 200;
//        else
//            driving_struct.velocity = 0;

//        vel_tick++;

//        driving_struct.velocity = 45;
        driving_struct.velocity = VelGen_Cnt(vel_up_cnt,new_rench_update,20);
        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
//        Sleep(30);
    }while(rench_desirable_differ_current != 0);

//    driving_struct.velocity = 0;
    mpc_vehicle->Move(1,0);
}


int CDriving::VelGen_Cnt(int _cnt, bool _update, int _thres)
{
    int min_vel = 30;
    int max_vel = 100;
    int velocity = 0;
    int cnt_thres = _thres;

    if(_update == true)
    {
        velocity = min_vel;
    }
    else if(_update == false && _cnt >= cnt_thres)
    {
        velocity = mpc_vehicle -> GetVel();
        if(velocity == 0)
        {
            velocity = 30;
        }
        else
            velocity++;
    }

    return velocity;

}

void CDriving::ParkingDistanceControl()
{
    DRIVING_STRUCT driving_struct;

    int _s_deg = 0;
    int _e_deg = 180;

    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_lrf_index -s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    long* lrf_distance_raw = new long[1081];

    vector<double> detected_panel_info;

    while(1)
    {
        if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
            std::cout << "LRF : GetLRFData Error" << std::endl;
        }
        else
        {
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            do{
                detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
            }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(1) == 0));


            double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

            if(panel_slope_atan2 < 0)
            {
                panel_slope_atan2 += 2.0*PI;
            }

            panel_slope_norm_x = -detected_panel_info.at(1);
            panel_slope_norm_y = detected_panel_info.at(0);


            double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

            if(panel_slope_norm < 0)
            {
                panel_slope_norm += 2.0*PI;
            }

            double panel_length = detected_panel_info.at(2);


            cout << "LRF panel status : " << mpc_velodyne->GetLRFPanelFindStatus() << endl;
            cout << "panel length : " << panel_length << endl;
            if((!mpc_velodyne->GetLRFPanelFindStatus()) || (panel_length <= 0.1))
            {

                cout << "can not find panel" << endl;
//                driving_struct.direction = UGV_move_right;
//                driving_struct.velocity= VelGen_parking_turn_right();
//                mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
//                Sleep(30);
//                continue;

                do{
                    detected_panel_info = mpc_velodyne->GetLMS511PanelInfo();
                }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(1) == 0));


                panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

                if(panel_slope_atan2 < 0)
                {
                    panel_slope_atan2 += 2.0*PI;
                }

                panel_slope_norm_x = -detected_panel_info.at(1);
                panel_slope_norm_y = detected_panel_info.at(0);


                panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if(panel_slope_norm < 0)
                {
                    panel_slope_norm += 2.0*PI;
                }

                panel_length = detected_panel_info.at(2);

                if((!mpc_velodyne->GetLMS511PanelFindStatus()) || (panel_length <= 0.1))
                {
                    driving_struct.direction = UGV_move_right;
                    driving_struct.velocity= VelGen_parking_turn_right();
                    mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
                    Sleep(30);
                    continue;
                }
            }

            if(panel_length > 0.10) // any panel
            {
                double panel_center_to_origin_x = -detected_panel_info.at(3);
                double panel_center_to_origin_y = -detected_panel_info.at(4);

                double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                if(panel_center_to_origin_slope < 0)
                {
                    panel_center_to_origin_slope += 2.0*PI;
                }

                double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                if (norm_to_origin_angle > PI)
                {
                    norm_to_origin_angle -= PI;
                }

                if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                {
                    panel_slope_norm_x = -panel_slope_norm_x;
                    panel_slope_norm_y = -panel_slope_norm_y;
                }

                double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if (panel_slope_norm_angle < 0)
                {
                    panel_slope_norm_angle += 2*PI;
                }

                //// Test section ----------------////////////////

                double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                if(angle_between_norm_and_panel_center_to_origin > PI)
                {
                    angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                }

                double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);

                //// ------------------------------////////////////
                double dist_to_ugv_center = 0;

                if(detected_panel_info.at(0) == 0)
                {
                    dist_to_ugv_center = detected_panel_info.at(3);
                }
                else
                {
                    double lrf_eq_a = detected_panel_info.at(1)/detected_panel_info.at(0);
                    double lrf_eq_b = detected_panel_info.at(4) - lrf_eq_a*detected_panel_info.at(3);

                    double dist_target_x = 0.4;
                    double dist_target_y = 0;

                    dist_to_ugv_center = abs(-lrf_eq_a*dist_target_x + dist_target_y - lrf_eq_b)/sqrt(lrf_eq_a*lrf_eq_a+1);
                }

                if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI)))
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity = VelGen_parking_turn_left();
                }
                else if ((panel_slope_norm_angle >= (PI)) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                {
                    driving_struct.direction = UGV_move_right;
                    driving_struct.velocity= VelGen_parking_turn_right();
                }
                else
                {
                    if (dist_to_ugv_center > (parking_dist+ parking_thres))
                    {
                        driving_struct.direction = UGV_move_forward;
                        driving_struct.velocity = 70;
                    }
                    else if (dist_to_ugv_center < (parking_dist - parking_thres))
                    {
                        driving_struct.direction = UGV_move_backward;
                        driving_struct.velocity = 70;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
        }
    }

    while(1)
    {
        if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
            std::cout << "LRF : GetLRFData Error" << std::endl;
        }
        else
        {
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            do{
                detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
            }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(1) == 0));

            double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

            if(panel_slope_atan2 < 0)
            {
                panel_slope_atan2 += 2.0*PI;
            }

            panel_slope_norm_x = -detected_panel_info.at(1);
            panel_slope_norm_y = detected_panel_info.at(0);


            double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

            if(panel_slope_norm < 0)
            {
                panel_slope_norm += 2.0*PI;
            }

            double panel_length = detected_panel_info.at(2);

            if(panel_length > 0.10) // any panel
            {
                double panel_center_to_origin_x = -detected_panel_info.at(3);
                double panel_center_to_origin_y = -detected_panel_info.at(4);

                double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                if(panel_center_to_origin_slope < 0)
                {
                    panel_center_to_origin_slope += 2.0*PI;
                }

                double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                if (norm_to_origin_angle > PI)
                {
                    norm_to_origin_angle -= PI;
                }

                if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                {
                    panel_slope_norm_x = -panel_slope_norm_x;
                    panel_slope_norm_y = -panel_slope_norm_y;
                }

                double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if (panel_slope_norm_angle < 0)
                {
                    panel_slope_norm_angle += 2*PI;
                }

                //// Test section ----------------////////////////

                double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                if(angle_between_norm_and_panel_center_to_origin > PI)
                {
                    angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                }

                double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);


                //// ------------------------------////////////////
                double dist_to_ugv_center = 0;

                if(detected_panel_info.at(0) == 0)
                {
                    dist_to_ugv_center = detected_panel_info.at(3);
                }
                else
                {
                    double lrf_eq_a = detected_panel_info.at(1)/detected_panel_info.at(0);
                    double lrf_eq_b = detected_panel_info.at(4) - lrf_eq_a*detected_panel_info.at(3);

                    double dist_target_x = 0.4;
                    double dist_target_y = 0;

                    dist_to_ugv_center = abs(-lrf_eq_a*dist_target_x + dist_target_y - lrf_eq_b)/sqrt(lrf_eq_a*lrf_eq_a+1);
                }

                if((panel_slope_norm_angle > (0.5*PI + (10.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI)))
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity = VelGen_parking_turn_left();
                }
                else if (((panel_slope_norm_angle < (0.5*PI - (10.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI)) && (panel_slope_norm_angle < 2*PI)))
                {
                    driving_struct.direction = UGV_move_right;
                    driving_struct.velocity = VelGen_parking_turn_right();
                }
                else
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity =0;
                    mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
                    break;
                }
            }
            mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
        }
    }

    Sleep(1000);
}

void CDriving::WRenchRecog(){

    WRENCH_RECOGNITION_STRUCT wrench_gecog = GetWrenchRecogOption();

    DriveByRenchRecog(wrench_gecog.desired_wrench_index, wrench_gecog.lrf_v_dst);
}

void CDriving::LocalizationOnPanel(){
    VEHICLE_LOCALIZATION_ON_PANEL_STRUCT local_panel = GetVehicleLocalOnPanelOption();

    DriveByVelodyne(local_panel.desired_h_dst);
}

void CDriving::ParkingDistanceControl(double _bias)
{
    DRIVING_STRUCT driving_struct;

    int _s_deg = 0;
    int _e_deg = 180;

    int s_lrf_index = (int)((_s_deg + 45) / ANGLE_RESOLUTION);
    int e_lrf_index = (int)((_e_deg + 45) / ANGLE_RESOLUTION);

    int number_of_point = e_lrf_index -s_lrf_index + 1;

    long* lrf_distance = new long[number_of_point];

    long* lrf_distance_raw = new long[1081];

    vector<double> detected_panel_info;

    while(1)
    {
        if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
            std::cout << "LRF : GetLRFData Error" << std::endl;
        }
        else
        {
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            do{
                detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
            }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(1) == 0));

            double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

            if(panel_slope_atan2 < 0)
            {
                panel_slope_atan2 += 2.0*PI;
            }

            panel_slope_norm_x = -detected_panel_info.at(1);
            panel_slope_norm_y = detected_panel_info.at(0);


            double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

            if(panel_slope_norm < 0)
            {
                panel_slope_norm += 2.0*PI;
            }

            double panel_length = detected_panel_info.at(2);

            if(panel_length > 0.10) // any panel
            {
                double panel_center_to_origin_x = -detected_panel_info.at(3);
                double panel_center_to_origin_y = -detected_panel_info.at(4);

                double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                if(panel_center_to_origin_slope < 0)
                {
                    panel_center_to_origin_slope += 2.0*PI;
                }

                double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                if (norm_to_origin_angle > PI)
                {
                    norm_to_origin_angle -= PI;
                }

                if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                {
                    panel_slope_norm_x = -panel_slope_norm_x;
                    panel_slope_norm_y = -panel_slope_norm_y;
                }

                double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if (panel_slope_norm_angle < 0)
                {
                    panel_slope_norm_angle += 2*PI;
                }

                //// Test section ----------------////////////////

                double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                if(angle_between_norm_and_panel_center_to_origin > PI)
                {
                    angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                }

                double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);


                //// ------------------------------////////////////
                double dist_to_ugv_center = 0;

                if(detected_panel_info.at(0) == 0)
                {
                    dist_to_ugv_center = detected_panel_info.at(3);
                }
                else
                {
                    double lrf_eq_a = detected_panel_info.at(1)/detected_panel_info.at(0);
                    double lrf_eq_b = detected_panel_info.at(4) - lrf_eq_a*detected_panel_info.at(3);

                    double dist_target_x = 0.4;
                    double dist_target_y = 0;

                    dist_to_ugv_center = abs(-lrf_eq_a*dist_target_x + dist_target_y - lrf_eq_b)/sqrt(lrf_eq_a*lrf_eq_a+1);
                }

                if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI)))
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity = VelGen_turn_left();
                }
                else if ((panel_slope_norm_angle >= (PI)) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                {
                    driving_struct.direction = UGV_move_right;
                    driving_struct.velocity = VelGen_turn_right();
                }
                else
                {
                    if (dist_to_ugv_center > (parking_dist + _bias + parking_thres))
                    {
                        driving_struct.direction = UGV_move_forward;
                        driving_struct.velocity = parking_forward;
                    }
                    else if (dist_to_ugv_center < (parking_dist + _bias - parking_thres))
                    {
                        driving_struct.direction = UGV_move_backward;
                        driving_struct.velocity = parking_backward;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
        }
    }

    while(1)
    {
        if(!mpc_drive_lrf->GetLRFData(lrf_distance_raw)){
            std::cout << "LRF : GetLRFData Error" << std::endl;
        }
        else
        {
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            do{
                detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
            }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(1) == 0));

            double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

            if(panel_slope_atan2 < 0)
            {
                panel_slope_atan2 += 2.0*PI;
            }

            panel_slope_norm_x = -detected_panel_info.at(1);
            panel_slope_norm_y = detected_panel_info.at(0);


            double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

            if(panel_slope_norm < 0)
            {
                panel_slope_norm += 2.0*PI;
            }

            double panel_length = detected_panel_info.at(2);

            if(panel_length > 0.10) // any panel
            {
                double panel_center_to_origin_x = -detected_panel_info.at(3);
                double panel_center_to_origin_y = -detected_panel_info.at(4);

                double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

                if(panel_center_to_origin_slope < 0)
                {
                    panel_center_to_origin_slope += 2.0*PI;
                }

                double norm_to_origin_angle = abs(panel_center_to_origin_slope - panel_slope_norm);

                if (norm_to_origin_angle > PI)
                {
                    norm_to_origin_angle -= PI;
                }

                if (acos((panel_slope_norm_x*panel_center_to_origin_x + panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
                {
                    panel_slope_norm_x = -panel_slope_norm_x;
                    panel_slope_norm_y = -panel_slope_norm_y;
                }

                double panel_slope_norm_angle = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if (panel_slope_norm_angle < 0)
                {
                    panel_slope_norm_angle += 2*PI;
                }

                //// Test section ----------------////////////////

                double panel_slope_norm_x_norm = panel_slope_norm_x/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);
                double panel_slope_norm_y_norm = panel_slope_norm_y/sqrt(panel_slope_norm_x*panel_slope_norm_x + panel_slope_norm_y*panel_slope_norm_y);

                double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

                if(angle_between_norm_and_panel_center_to_origin > PI)
                {
                    angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
                }

                double panel_way_anchor_x = detected_panel_info.at(3) + panel_slope_norm_x_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);
                double panel_way_anchor_y = detected_panel_info.at(4) + panel_slope_norm_y_norm*sqrt(panel_center_to_origin_x*panel_center_to_origin_x + panel_center_to_origin_y*panel_center_to_origin_y)*cos(angle_between_norm_and_panel_center_to_origin);

                panel_way_x = panel_way_anchor_x + side_center_margin*(-panel_slope_norm_y_norm);
                panel_way_y = panel_way_anchor_y + side_center_margin*(panel_slope_norm_x_norm);


                //// ------------------------------////////////////
                double dist_to_ugv_center = 0;

                if(detected_panel_info.at(0) == 0)
                {
                    dist_to_ugv_center = detected_panel_info.at(3);
                }
                else
                {
                    double lrf_eq_a = detected_panel_info.at(1)/detected_panel_info.at(0);
                    double lrf_eq_b = detected_panel_info.at(4) - lrf_eq_a*detected_panel_info.at(3);

                    double dist_target_x = 0.4;
                    double dist_target_y = 0;

                    dist_to_ugv_center = abs(-lrf_eq_a*dist_target_x + dist_target_y - lrf_eq_b)/sqrt(lrf_eq_a*lrf_eq_a+1);
                }

                if((panel_slope_norm_angle > (0.5*PI + (10.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI)))
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity = VelGen_turn_left();
                }
                else if (((panel_slope_norm_angle < (0.5*PI - (10.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI)) && (panel_slope_norm_angle < 2*PI)))
                {
                    driving_struct.direction = UGV_move_right;
                    driving_struct.velocity = VelGen_turn_right();
                }
                else
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity =0;
                    mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
                    break;
                }
            }
            mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
        }
    }

    Sleep(1000);
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

    int count = 0;

    do{
        int direction = 0;
        double current_a_inlier_deg = 0;
        double current_a_virtual_deg = 0;

        double a_deg_boundary = 0;
        double a_deg_virtual_boundary = 0;

        double horizen_distance = 0;

        double s_virture_deg = 0;
        double e_virture_deg = 0;

        double s_inlier_deg = 0;
        double e_inlier_deg = 0;

        mpc_rgb_d->GetHorizenDistance(inlier_distance, horizen_distance, s_inlier_deg, e_inlier_deg,
                                      s_virture_deg, e_virture_deg, lrf_vehicle.s_deg, lrf_vehicle.e_deg);

        current_a_inlier_deg = (s_inlier_deg + e_inlier_deg) / 2;
        current_a_virtual_deg = (s_virture_deg + e_virture_deg) / 2;;

        a_deg_boundary = lrf_vehicle.desired_avr_inlier_deg - current_a_inlier_deg;
        a_deg_virtual_boundary = lrf_vehicle.desired_avr_inlier_deg - current_a_virtual_deg;

        lrf_vehicle.horizen_distance = horizen_distance;

        lrf_vehicle.s_inlier_deg = s_inlier_deg;
        lrf_vehicle.e_inlier_deg = e_inlier_deg;

        lrf_vehicle.s_virtual_deg = s_virture_deg;
        lrf_vehicle.e_virtual_deg = e_virture_deg;

        emit SignalLRFVehicleHorizenStruct(lrf_vehicle);

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

        count = 0;

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


bool CDriving::DrivieByOdometer(double _heading_constraint, double _distance_constraint)
{
    DRIVING_STRUCT driving_struct;

    double heading_control_margin = 2.0/180.0*PI;


    mpc_vehicle->InitEncoder();

    int target_encoder_value = mpc_vehicle->CalcDistToEnc_m(_distance_constraint);

    target_encoder_value = target_encoder_value * 1000;
    int current_encoder_value;
    do{

        current_encoder_value = 0.5*(mpc_vehicle->GetEncoderValue().at(0) + mpc_vehicle->GetEncoderValue().at(1));
        double current_heading = euler_angles.at(2);

        if(current_heading < 0)
        {
            current_heading += 2*PI;
        }

        if(_heading_constraint < PI)
        {
            if(((_heading_constraint + heading_control_margin) < current_heading ) &&(current_heading < (_heading_constraint + PI)) )
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = 100;
            }
            else if(((current_heading >= 0) && (current_heading < (_heading_constraint - heading_control_margin))) || ((current_heading > _heading_constraint + PI) && (current_heading < 2*PI)))
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = 100;
            }
            else
            {
                driving_struct.direction = UGV_move_backward;
                driving_struct.velocity = 100;
            }
        }
        else
        {
            if( ((current_heading > (_heading_constraint + heading_control_margin)) && (current_heading < 2*PI)) || ((current_heading >= 0) && (current_heading < _heading_constraint - PI) ))
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = 100;
            }
            else if((current_heading >=  _heading_constraint - PI) &&(current_heading < _heading_constraint - heading_control_margin) )
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = 100;
            }
            else
            {
                driving_struct.direction = UGV_move_backward;
                driving_struct.velocity = 100;
            }
        }
        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    }while(current_encoder_value < target_encoder_value);

    return true;
}

bool CDriving::InitialSearching(double _StraightDistance)
{
    DRIVING_STRUCT driving_struct;

    double heading_control_margin = 2.0/180.0*PI;

    double _heading_constraint = euler_angles.at(2);

    if(_heading_constraint < 0)
    {
        _heading_constraint += 2*PI;
    }

    double _heading_constraint_left = _heading_constraint - 0.5*PI;

    if(_heading_constraint_left < 0)
    {
        _heading_constraint_left += 2*PI;
    }
    else if (_heading_constraint_left >= 2*PI)
    {
        _heading_constraint_left -= 2*PI;
    }

    double _heading_constraint_right = _heading_constraint + 0.5*PI;

    if(_heading_constraint_right < 0)
    {
        _heading_constraint_right += 2*PI;
    }
    else if (_heading_constraint_right >= 2*PI)
    {
        _heading_constraint_right -= 2*PI;
    }


    int target_encoder_value = mpc_vehicle->CalcDistToEnc_m(_StraightDistance);

    target_encoder_value = 0.5*(mpc_vehicle->GetEncoderValue().at(0) + mpc_vehicle->GetEncoderValue().at(1)) + target_encoder_value * 1000;

    int current_encoder_value;

    do{
        if(panel_found)
            return true;

        current_encoder_value = 0.5*(mpc_vehicle->GetEncoderValue().at(0) + mpc_vehicle->GetEncoderValue().at(1));
        double current_heading = euler_angles.at(2);

        if(current_heading < 0)
        {
            current_heading += 2*PI;
        }

        if(_heading_constraint < PI)
        {
            if(((_heading_constraint + heading_control_margin) < current_heading ) &&(current_heading < (_heading_constraint + PI)) )
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_turn_left();
            }
            else if(((current_heading >= 0) && (current_heading < (_heading_constraint - heading_control_margin))) || ((current_heading > _heading_constraint + PI) && (current_heading < 2*PI)))
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = VelGen_turn_right();
            }
            else
            {
                driving_struct.direction = UGV_move_forward;
                driving_struct.velocity = 200;
            }
        }
        else
        {
            if( ((current_heading > (_heading_constraint + heading_control_margin)) && (current_heading < 2*PI)) || ((current_heading >= 0) && (current_heading < _heading_constraint - PI) ))
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_turn_left();
            }
            else if((current_heading >=  _heading_constraint - PI) &&(current_heading < _heading_constraint - heading_control_margin) )
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = VelGen_turn_right();
            }
            else
            {
                driving_struct.direction = UGV_move_forward;
                driving_struct.velocity = 200;
            }
        }
        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    }while(current_encoder_value < target_encoder_value);


    driving_struct.direction = UGV_move_forward;
    driving_struct.velocity = 0;
    mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    Sleep(1000);

    // Turn left 90 degree

    while(1)
    {
        double current_heading = euler_angles.at(2);

        if(current_heading < 0)
        {
            current_heading += 2*PI;
        }

        if(_heading_constraint_left < PI)
        {
            if(((_heading_constraint_left + heading_control_margin) < current_heading ) &&(current_heading < (_heading_constraint_left + PI)) )
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_turn_left();
            }
            else if(((current_heading >= 0) && (current_heading < (_heading_constraint_left - heading_control_margin))) || ((current_heading > _heading_constraint_left + PI) && (current_heading < 2*PI)))
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = VelGen_turn_right();
            }
            else
            {
                break;
            }
        }
        else
        {
            if( ((current_heading > (_heading_constraint_left + heading_control_margin)) && (current_heading < 2*PI)) || ((current_heading >= 0) && (current_heading < _heading_constraint_left - PI) ))
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_turn_left();
            }
            else if((current_heading >=  _heading_constraint_left - PI) &&(current_heading < _heading_constraint_left - heading_control_margin) )
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = VelGen_turn_right();
            }
            else
            {
                break;
            }
        }
        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
    }

    driving_struct.direction = UGV_move_left;
    driving_struct.velocity = 0;
    mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    if(panel_found)
        return true;


    // Turn right 90 degree

    while(1)
    {

        double current_heading = euler_angles.at(2);

        if(current_heading < 0)
        {
            current_heading += 2*PI;
        }

        if(_heading_constraint_right < PI)
        {
            if(((_heading_constraint_right + heading_control_margin) < current_heading ) &&(current_heading < (_heading_constraint_right + PI)) )
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_turn_left();
            }
            else if(((current_heading >= 0) && (current_heading < (_heading_constraint_right - heading_control_margin))) || ((current_heading > _heading_constraint_right + PI) && (current_heading < 2*PI)))
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = VelGen_turn_right();
            }
            else
            {
                break;
            }
        }
        else
        {
            if( ((current_heading > (_heading_constraint_right + heading_control_margin)) && (current_heading < 2*PI)) || ((current_heading >= 0) && (current_heading < _heading_constraint_right - PI) ))
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity = VelGen_turn_left();
            }
            else if((current_heading >=  _heading_constraint_right - PI) &&(current_heading < _heading_constraint_right - heading_control_margin) )
            {
                driving_struct.direction = UGV_move_right;
                driving_struct.velocity = VelGen_turn_right();
            }
            else
            {
                break;
            }
        }
        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
    }


    if(panel_found)
    {
        return true;
    }
    else
    {
        return false;
    }

}

void CDriving::PanelFrontDistanceControlByLMS511()
{
    DRIVING_STRUCT driving_struct;

    double parking_dist = 0.8;
    double parking_thres = 0.05;
    double dist_to_ugv_center = 0;
    vector<double> panel_info;
    while(abs(dist_to_ugv_center-parking_dist) > parking_thres)
    {
        // Get panel distance by lms511
        do{
//            panel_info = mpc_velodyne->GetLMS511PanelInfo();
            panel_info = mpc_velodyne->GetLRFPanelInfo();
            cout << "panel info 0" << panel_info.at(0) << ", panel info 1 " << panel_info.at(1) << endl;
        }while((panel_info.at(0) == 0) && (panel_info.at(1) == 0));


        ///////////////////////////////////////////////////
        double panel_slope_atan2 = atan2(panel_info.at(1),panel_info.at(0));

        if(panel_slope_atan2 < 0)
        {
            panel_slope_atan2 += 2.0*PI;
        }

        double lms511_panel_slope_norm_x = -panel_info.at(1);
        double lms511_panel_slope_norm_y = panel_info.at(0);


        double lms511_panel_slope_norm = atan2(lms511_panel_slope_norm_y,lms511_panel_slope_norm_x);

        if(lms511_panel_slope_norm < 0)
        {
            lms511_panel_slope_norm += 2.0*PI;
        }

        double panel_center_to_origin_x = -panel_info.at(3);
        double panel_center_to_origin_y = -panel_info.at(4);

        double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

        if(panel_center_to_origin_slope < 0)
        {
            panel_center_to_origin_slope += 2.0*PI;
        }

        double norm_to_origin_angle = abs(panel_center_to_origin_slope - lms511_panel_slope_norm);

        if (norm_to_origin_angle > PI)
        {
            norm_to_origin_angle -= PI;
        }

        if (acos((lms511_panel_slope_norm_x*panel_center_to_origin_x + lms511_panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
        {
            lms511_panel_slope_norm_x = -lms511_panel_slope_norm_x;
            lms511_panel_slope_norm_y = -lms511_panel_slope_norm_y;
        }

        double panel_slope_norm_angle = atan2(lms511_panel_slope_norm_y,lms511_panel_slope_norm_x);

        if (panel_slope_norm_angle < 0)
        {
            panel_slope_norm_angle += 2*PI;
        }

        double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

        if(angle_between_norm_and_panel_center_to_origin > PI)
        {
            angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
        }



        if(panel_info.at(0) == 0)
        {
            dist_to_ugv_center = panel_info.at(3);
        }
        else
        {
            double lms511_eq_a = panel_info.at(1)/panel_info.at(0);
            double lms511_eq_b = panel_info.at(4) - lms511_eq_a*panel_info.at(3);

            double dist_target_x = 0.4;
            double dist_target_y = 0;

            dist_to_ugv_center = abs(-lms511_eq_a*dist_target_x + dist_target_y - lms511_eq_b)/sqrt(lms511_eq_a*lms511_eq_a+1);
        }


        ///////////////////////////////////////////////

        if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI)))
        {
            driving_struct.direction = UGV_move_left;
            driving_struct.velocity = 100;
        }
        else if ((panel_slope_norm_angle > (PI)) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
        {
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity = 100;
        }
        else
        {
            if(dist_to_ugv_center > parking_dist)
            {
                driving_struct.direction = UGV_move_forward;
                driving_struct.velocity = 50;
            }
            else
            {
                driving_struct.direction = UGV_move_backward;
                driving_struct.velocity = 60;
            }
        }


        cout << "panel slope norm x : " << panel_slope_norm_x << endl;
        cout << "panel slope norm y : " << panel_slope_norm_y << endl;
        cout << "panel_slope_norm_angle : " << panel_slope_norm_angle << endl;


        mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
    }

    do{
        // Get panel distance by lms511
        do{
            panel_info = mpc_velodyne->GetLMS511PanelInfo();
        }while((panel_info.at(0) == 0) && (panel_info.at(1) == 1));


        ///////////////////////////////////////////////////
        double panel_slope_atan2 = atan2(panel_info.at(1),panel_info.at(0));

        if(panel_slope_atan2 < 0)
        {
            panel_slope_atan2 += 2.0*PI;
        }

        double lms511_panel_slope_norm_x = -panel_info.at(1);
        double lms511_panel_slope_norm_y = panel_info.at(0);


        double lms511_panel_slope_norm = atan2(lms511_panel_slope_norm_y,lms511_panel_slope_norm_x);

        if(lms511_panel_slope_norm < 0)
        {
            lms511_panel_slope_norm += 2.0*PI;
        }

        double panel_center_to_origin_x = -panel_info.at(3);
        double panel_center_to_origin_y = -panel_info.at(4);

        double panel_center_to_origin_slope = atan2(panel_center_to_origin_y,panel_center_to_origin_x);

        if(panel_center_to_origin_slope < 0)
        {
            panel_center_to_origin_slope += 2.0*PI;
        }

        double norm_to_origin_angle = abs(panel_center_to_origin_slope - lms511_panel_slope_norm);

        if (norm_to_origin_angle > PI)
        {
            norm_to_origin_angle -= PI;
        }

        if (acos((lms511_panel_slope_norm_x*panel_center_to_origin_x + lms511_panel_slope_norm_y*panel_center_to_origin_y)/(sqrt(panel_center_to_origin_x*panel_center_to_origin_x+panel_center_to_origin_y*panel_center_to_origin_y))) > 0.5*PI)
        {
            lms511_panel_slope_norm_x = -lms511_panel_slope_norm_x;
            lms511_panel_slope_norm_y = -lms511_panel_slope_norm_y;
        }

        double panel_slope_norm_angle = atan2(lms511_panel_slope_norm_y,lms511_panel_slope_norm_x);

        if (panel_slope_norm_angle < 0)
        {
            panel_slope_norm_angle += 2*PI;
        }

        double angle_between_norm_and_panel_center_to_origin = abs(panel_slope_norm_angle - panel_center_to_origin_slope);

        if(angle_between_norm_and_panel_center_to_origin > PI)
        {
            angle_between_norm_and_panel_center_to_origin = 2*PI - angle_between_norm_and_panel_center_to_origin;
        }



        if(panel_info.at(0) == 0)
        {
            dist_to_ugv_center = panel_info.at(3);
        }
        else
        {
            double lms511_eq_a = panel_info.at(1)/panel_info.at(0);
            double lms511_eq_b = panel_info.at(4) - lms511_eq_a*panel_info.at(3);

            double dist_target_x = 0.4;
            double dist_target_y = 0;

            dist_to_ugv_center = abs(-lms511_eq_a*dist_target_x + dist_target_y - lms511_eq_b)/sqrt(lms511_eq_a*lms511_eq_a+1);
        }


        ///////////////////////////////////////////////
        cout << "panel slope angle : " << panel_slope_norm_angle << endl;

        if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI)))
        {
            driving_struct.direction = UGV_move_left;
            driving_struct.velocity = 95;
        }
        else if (((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI)) && (panel_slope_norm_angle < 2*PI)))
        {
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity = 95;
        }
        else
        {
            break;
        }

        mpc_vehicle->Move(driving_struct.direction,driving_struct.velocity);
    }while(true);


}

double CDriving::CalcDistErrorToCheckPoint(double _dist_m)
{
    Gpspoint cur_gps;
    cur_gps = mpc_gps->GetCurrentGPS();
    cur_gps.lat = cur_gps.lat;
    cur_gps.lon = cur_gps.lon;

    Gpspoint init_gps;
    init_gps = mpc_gps->GetInitGPS();
    init_gps.lat = init_gps.lat;
    init_gps.lon = init_gps.lon;

    Gpspoint target_gps;
    double dist_km = _dist_m /1000.0;
    target_gps = mpc_gps->CalcGpspoint(0,dist_km,init_gps);

    double dist_init2cur_forcheck = mpc_gps->DistCalc_Gps2Gps(init_gps,cur_gps);
    double dist_init2target_forcheck = mpc_gps->DistCalc_Gps2Gps(init_gps,target_gps);

    double dist_error = mpc_gps->DistCalc_Gps2Gps(cur_gps,target_gps);

    if(dist_init2cur_forcheck > dist_init2target_forcheck) // odometry moves more than target gps point
        dist_error = abs(dist_error) * -1;
    else                                                    // odometry moves less than target gps point
        dist_error = abs(dist_error);

    return dist_error;
}

bool CDriving::DrivingMissionManager()
{
    // Init encoder
    mpc_vehicle->InitEncoder();

    aligned_initial_heading = euler_angles.at(2);
    if(aligned_initial_heading < 0)
    {
        aligned_initial_heading += 2*PI;
    }
    arena_aligned_compensate = true;



    double dist_m_to_p1 = 20.0;
    DrivieByOdometer(aligned_initial_heading, 20.0);

    m_dist_error_from_p1_km = CalcDistErrorToCheckPoint(dist_m_to_p1);

//    Sleep(1000);

    // Check Panel Found
    if(panel_found)
    {
//        DriveToPanel();
    }
    else
    {
//        double dist_m_to_p1 = 20.0;
//        DrivieByOdometer(aligned_initial_heading, 20.0);

//        m_dist_error_from_p1_km = CalcDistErrorToCheckPoint(dist_m_to_p1);
    }
    return true;
}

CGPS* CDriving::GetGPS()
{
    return mpc_gps;
}

// Slots
void CDriving::SlotIMUEuler(vector<double> _euler_angles)
{
    mpc_imu->mtx_imu.lock();
    if(IMU_update_finished)
    {
        IMU_update_finished = false;
        euler_angles = _euler_angles;

        double current_heading = euler_angles.at(2);
        if(current_heading < 0)
        {
            current_heading += 2*PI;
        }
        emit SignalVehicleHeading(current_heading);

        if(arena_aligned_compensate)
        {
//            vector<int> encoder_new = mpc_vehicle->GetEncoderValueRaw();
//            if(encoder_new.size() != 2)
//            {
//                return;
//            }
//            else
//            {
//                past_encoder_value = current_encoder_value;
//                current_encoder_value = encoder_new;
//            }
//            past_encoder_value = current_encoder_value;
//            do
//            {
//                current_encoder_value = mpc_vehicle->GetEncoderValueRaw();
//            }while(current_encoder_value.size() != 2);
            if((abs(current_encoder_value.at(0)) - abs(past_encoder_value.at(0)))*(abs(current_encoder_value.at(1)) - abs(past_encoder_value.at(1))) < 0) // vehicle is turning
            {
                encoder_bias_update = true;
            }
            else
            {
                if(encoder_bias_update)
                {
                    arena_move_coordinate_old.at(0) = arena_move_coordinate.at(0);
                    arena_move_coordinate_old.at(1) = arena_move_coordinate.at(1);
                    encoder_bias_1 = current_encoder_value.at(0);
                    encoder_bias_2 = current_encoder_value.at(1);
                    encoder_bias_update = false;
                }
                double encoder_change_mean = 0.5*(double)(abs(current_encoder_value.at(0) - encoder_bias_1) + abs(current_encoder_value.at(1)- encoder_bias_2));
                if( ((current_encoder_value.at(0) - encoder_bias_1) < 0) &&  ((current_encoder_value.at(1) - encoder_bias_2) > 0))
                {
                    //move forward
                    total_move_distance = (encoder_change_mean/1000.0 + 28.2715)/93.1531 - 0.30349;
                }
                else
                {
                    //move backward
                    total_move_distance = -((encoder_change_mean/1000.0 + 28.2715)/93.1531 - 0.30349);
                }
                arena_move_coordinate.at(0) = arena_move_coordinate_old.at(0) - total_move_distance*cos(current_heading - aligned_initial_heading);
                arena_move_coordinate.at(1) = arena_move_coordinate_old.at(1) + total_move_distance*sin(current_heading - aligned_initial_heading);
            }

            aligned_heading_to_current_differ = current_heading - aligned_initial_heading;

            vector<vector<double>> new_arena_info;

            for(int i = 0; i < 4;i++)
            {
                vector<double> new_arena_info_element = {0,0};
                new_arena_info_element.at(0) = (m_arena_info.at(i)).at(0)*cos(aligned_heading_to_current_differ) - (m_arena_info.at(i)).at(1)*sin(aligned_heading_to_current_differ);
                new_arena_info_element.at(1) = (m_arena_info.at(i)).at(0)*sin(aligned_heading_to_current_differ) + (m_arena_info.at(i)).at(1)*cos(aligned_heading_to_current_differ);
                new_arena_info.push_back(new_arena_info_element);
            }
            mpc_velodyne->SetArenaBoundary(new_arena_info,-arena_move_coordinate.at(0),-arena_move_coordinate.at(1),aligned_heading_to_current_differ);
        }
        IMU_update_finished = true;
    }
    mpc_imu->mtx_imu.unlock();
}

void CDriving::SlotIMULinearAccel(mip_filter_linear_acceleration _linear_accel)
{
    linear_accel = _linear_accel;
}

void CDriving::SlotIMUTimestamp(mip_ahrs_internal_timestamp _time_stamp)
{
    time_stamp = _time_stamp;
}

void CDriving::SlotIMUDeltaVelocity(mip_ahrs_delta_velocity _delta_velocity)
{
    delta_velocity = _delta_velocity;
}

void CDriving::SlotVelodynePanelFound(bool _panel_found)
{
    panel_found = _panel_found;
    vector<int> encoder_new = mpc_vehicle->GetEncoderValueRaw();
    if(encoder_new.size() != 2)
    {
        return;
    }
    else
    {
        past_encoder_value = current_encoder_value;
        current_encoder_value = encoder_new;
    }
}

void CDriving::SlotLMS511UpdatePoints(vector<vector<double>> _point_list)
{

}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CDriving::run(){

    switch (m_main_fnc_index) {

    case DRIVE_INX_PARKING_RETRY:
        ParkingRetry();
        break;
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
    case DRIVE_INX_LOCAL_ON_PANE:
        LocalizationOnPanel();
        break;
    case DRIVE_INX_WRENCH_RECOG:
        WRenchRecog();
        break;
    default:
        break;

    }
}

//

int CDriving::VelGen_Cnt(int _cnt, bool _update)
{
    int min_vel = 50;
    int max_vel = 100;
    int velocity = 0;
    int cnt_thres = 5000;

    if(_update == true)
    {
        velocity = min_vel;
    }
    else if(_update == false && _cnt >= cnt_thres)
    {
        velocity = mpc_vehicle->GetVel();
        velocity++;
    }

    return velocity;
}
