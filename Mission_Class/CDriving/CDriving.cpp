#include "CDriving.h"

CDriving::CDriving(){

}

CDriving::CDriving(CIMU* _p_imu, CGPS* _p_gps, CLRF* _p_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne){

    mpc_imu = _p_imu;
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


//GPS
bool CDriving::ConnectGPS()
{
    if(!mpc_gps->GpsInit())
    {
        std::cout << "Fail to Connection GPS" << std::endl;
        return false;
    }
    else
        return true;
}
bool CDriving::IsGPSConnected(){
    return mpc_gps->IsGPSConnected();
}

void CDriving::SetInitGPSpoint()
{
    mpc_gps->SetInitGPS();
    mpc_gps->CalcGroundGpspoint();
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

    if(heading_error < -10.0 / 180.0 * PI) // waypoint on the left
    {
        dir = UGV_move_left;
        return dir;
    }
    else if(heading_error > 10.0 / 180 * PI) // waypoint on the right
    {
        dir = UGV_move_right;
        return dir;
    }
    else if(dist_error < -0.3) // waypoint on the back
    {
        dir = UGV_move_backward;
        return dir;
    }
    else if(dist_error > 0.3) // waypoint on the front
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

bool CDriving::DriveToPanel(){

    DRIVING_STRUCT driving_struct;

    if(!mpc_vehicle->Connect()){
        std::cout << "Fail to Connection Vehicle" << std::endl;
        return false;
    }

    if(!mpc_gps->port->isOpen())
    {
        std::cout << "Fail to Connection GPS" << std::endl;
        return false;
    }

    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_DRIVING);

    do{

        vector<double> way_point_error = GetWaypointError(mpc_velodyne->GetWaypoint().at(0),mpc_velodyne->GetWaypoint().at(1));

        switch(GetParkingControl(way_point_error))
        {
        case 3: // turn left
            driving_struct.direction = UGV_move_left;
            driving_struct.velocity =75;
            break;
        case 4: // turn right
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity =75;
            break;
        case 1: // go forward
            driving_struct.direction = UGV_move_forward;
            driving_struct.velocity =70;
            break;
        case 2: // go backward
            driving_struct.direction = UGV_move_backward;
            driving_struct.velocity =70;
            break;
        default:

            break;
        }


//        int heading_control_flag = GetPanelHeadingError();
//        vector<double> panel_loc_info = mpc_velodyne->GetPanelCenterLoc();

//        double distance = panel_loc_info.at(2);
//        int test_vel;
//        if(distance < 0.6)
//            test_vel =0;
//        else if(distance >= 0.6)
//            test_vel = 65;
//        else
//            test_vel =0;

//        switch(heading_control_flag)
//        {
//        case -1: // turn left
//            driving_struct.direction = UGV_move_left;
//            driving_struct.velocity =75;
//            break;
//        case 1: // turn right
//            driving_struct.direction = UGV_move_right;
//            driving_struct.velocity =75;
//            break;
//        case 0: // Go straight
//            driving_struct.direction = UGV_move_forward;
//            driving_struct.velocity =test_vel;
//            break;
//        default:

//            break;
//        }

        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    }while(driving_struct.driving_mission);

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
    double side_center_margin = 0.8;
    double front_center_margin = 0.3;

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
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            if(!mpc_velodyne->GetLRFPanelFindStatus())
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity =85;
            }
            else
            {

                detected_panel_info = mpc_velodyne->GetLRFPanelInfo();

                double panel_slope = atan(detected_panel_info.at(1)/detected_panel_info.at(0));

                double panel_slope_atan2 = atan2(detected_panel_info.at(1),detected_panel_info.at(0));

                if(panel_slope_atan2 < 0)
                {
                    panel_slope_atan2 += 2.0*PI;
                }

                double panel_slope_norm_x = -detected_panel_info.at(1);
                double panel_slope_norm_y = detected_panel_info.at(0);

                double panel_slope_norm = atan2(panel_slope_norm_y,panel_slope_norm_x);

                if(panel_slope_norm < 0)
                {
                    panel_slope_norm += 2.0*PI;
                }



                double panel_length = detected_panel_info.at(2);

                if((panel_length > 0.5 - panel_length_margin)&&(panel_length < 0.5 + panel_length_margin)) // side
                {

                    double panel_center_to_origin_x = -(detected_panel_info.at(3) - side_center_margin);
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

                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_slope_norm_y - panel_center_to_origin_y*panel_slope_norm_x;


                    if(outer_product_panel_center_norm  > 0)
                    {
                        if(panel_slope > (5.0/180.0*PI) || (abs(panel_slope) > (75.0/180.0*PI)))
                        {
                            driving_struct.direction = UGV_move_left;
                            driving_struct.velocity =85;
                        }
                        else if(panel_slope < - (5.0/180.0*PI))
                        {
                            driving_struct.direction = UGV_move_right;
                            driving_struct.velocity =85;
                        }
                        else if(outer_product_panel_center_norm  > 0)
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity =100;
                        }
                    }
                    else
                    {
//                        driving_struct.direction = UGV_move_differ_left;
//                        driving_struct.velocity =120;
                        driving_struct.direction = UGV_move_forward;
                        driving_struct.velocity =100;
//                        driving_struct.direction = UGV_move_left;
//                        driving_struct.velocity =85;
                    }
                }
                else if(panel_length > 2.0)
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity =85;
                }
                else// front or back
                {
                    double panel_center_to_origin_x = -(detected_panel_info.at(3) - front_center_margin);
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

                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_slope_norm_y - panel_center_to_origin_y*panel_slope_norm_x;


                    double line_distance_to_origin;

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

                    if ( (line_distance_to_origin < (side_center_margin + 0.25)) && (abs(panel_slope) > (75.0/180.0*PI)))
                    {
                        driving_struct.direction = UGV_move_forward;
                        driving_struct.velocity =100;
                    }
                    else
                    {
                        if(outer_product_panel_center_norm  > 0)
                        {
                            if(panel_slope > (5.0/180.0*PI) || (abs(panel_slope) > (75.0/180.0*PI)))
                            {
                                driving_struct.direction = UGV_move_left;
                                driving_struct.velocity =85;
                            }
                            else if(panel_slope < - (5.0/180.0*PI))
                            {
                                driving_struct.direction = UGV_move_right;
                                driving_struct.velocity =85;
                            }
                            else if(outer_product_panel_center_norm  > 0)
                            {
                                driving_struct.direction = UGV_move_forward;
                                driving_struct.velocity =100;
                            }

                        }
                        else
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity =100;
                        }
                    }
                }

            }
            cout<<driving_struct.direction << endl;
            mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
        }
    }while(driving_struct.driving_mission);


//    do{
//        if(!mpc_velodyne->GetPanelFindStatus())
//        {
//            if(mpc_velodyne->GetUGVTurnDirection())
//            {
//                driving_struct.direction = UGV_move_left;
//                driving_struct.velocity =75;
//            }
//            else
//            {
//                driving_struct.direction = UGV_move_right;
//                driving_struct.velocity =75;
//            }
//        }
//        else
//        {
//            vector<double> way_point_error = GetWaypointError(mpc_velodyne->GetWaypoint().at(0),mpc_velodyne->GetWaypoint().at(1));
//            cout<<"heading : " << way_point_error.at(0) <<", distance : "<<way_point_error.at(1) << endl;

//            switch(GetParkingControl(way_point_error))
//            {
//            case 3: // turn left
//                driving_struct.direction = UGV_move_left;
//                driving_struct.velocity =75;
//                break;
//            case 4: // turn right
//                driving_struct.direction = UGV_move_right;
//                driving_struct.velocity =75;
//                break;
//            case 1: // go forward
//                driving_struct.direction = UGV_move_forward;
//                driving_struct.velocity =70;
//                break;
//            case 2: // go backward
//                driving_struct.direction = UGV_move_backward;
//                driving_struct.velocity =70;
//                break;
//            default:

//                break;
//            }
//        }


//        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);
//        QThread::usleep(30);

//    }while(driving_struct.driving_mission);

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
