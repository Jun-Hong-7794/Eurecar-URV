#include "CDriving.h"

vector<vector<double>> lrf_panelpoint = {{0.5,0.0},{0.0,0.375}};
vector<vector<double>> lrf_waypoint = {{1.5,1.425}, {0.0,1.425}, {-1.5,1.425}, {-1.5,0.0}};
double lrf_waypoint_update_thres = 0.2;



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

CIMU* CDriving::GetIMU(){
    return mpc_imu;
}

CVelodyne* CDriving::GetVelodyne(){
    return mpc_velodyne;
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

bool CDriving::DriveToPanel(){

    DRIVING_STRUCT driving_struct;

    if(!mpc_vehicle->Connect()){
        std::cout << "Fail to Connection Vehicle" << std::endl;
        return false;
    }


//    if(!mpc_gps->port->isOpen())
//    {
//        std::cout << "Fail to Connection GPS" << std::endl;
//        return false;
//    }

    cout << "Driving Start " << endl;
    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_DRIVING);

    do{

        vector<double> way_point_error = GetWaypointError(mpc_velodyne->GetPanelCenterLoc().at(0) + 1.35,mpc_velodyne->GetPanelCenterLoc().at(1));

        if(!((mpc_velodyne->GetPanelCenterLoc().at(0) == 0) && (mpc_velodyne->GetPanelCenterLoc().at(0) == 0)))
        {
//            double panel_center_to_ugv = sqrt(mpc_velodyne->GetPanelCenterLoc().at(0)*mpc_velodyne->GetPanelCenterLoc().at(0) + mpc_velodyne->GetPanelCenterLoc().at(1)*mpc_velodyne->GetPanelCenterLoc().at(1));

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
            driving_struct.velocity =100;
            break;
        case 4: // turn right
            driving_struct.direction = UGV_move_right;
            driving_struct.velocity =100;
            cout << "turn right" << endl;
            break;
        case 1: // go forward
            driving_struct.direction = UGV_move_forward;
            driving_struct.velocity =100;
            break;
        case 2: // go backward
            driving_struct.direction = UGV_move_backward;
            driving_struct.velocity =100;
            break;
        default:
            driving_struct.direction = UGV_move_backward;
            driving_struct.velocity =0;
            break;
        }

        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

    }while(driving_struct.driving_mission);
    cout << "driving mision done"<< endl;

    mpc_vehicle->Move(UGV_move_backward, 0);

    mpc_velodyne->SetVelodyneMode(VELODYNE_MODE_PARKING);

    Sleep(5000);

    //heading compensate before parking


    double dist_arr[6] ={0,0,0,0,0,0};
    double min_dist = 1000000.;
    int min_dist_point_index = -1;
    int panel_left_cnt =1;
    int panel_right_cnt =1;


    for(int i=0; i<6; i++)
    {
        dist_arr[i] = 100*(mpc_velodyne->GetPCL()->panelpoint_cloud->points[i].x * mpc_velodyne->GetPCL()->panelpoint_cloud->points[i].x + mpc_velodyne->GetPCL()->panelpoint_cloud->points[i].y * mpc_velodyne->GetPCL()->panelpoint_cloud->points[i].y);

        if(dist_arr[i] < min_dist)
        {
            min_dist = dist_arr[i];
            min_dist_point_index = i;
        }
        if(mpc_velodyne->GetPCL()->panelpoint_cloud->points[i].y < 0 )
            panel_left_cnt = panel_left_cnt + 1;
        else
            panel_right_cnt = panel_right_cnt + 1;
    }

    cout << "index of min distance : " << min_dist_point_index << endl;



    vector<double> rec_imu_data;
    vector<double> rec_imu_data_tmp;
    rec_imu_data = mpc_velodyne->GetIMUData();


    double rotate_rad;

    if(panel_left_cnt == 6)
    {
        cout << "all panel left"<< endl;
        if(min_dist_point_index == 5 )
        {
            double empty_check = 0;
            do{
                empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
            }while(empty_check == 0);
            rotate_rad = atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x));
        }
        else
        {
            cout << "all panel left and not in index 5"<< endl;
            double empty_check = 0;
            do{
                empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
            }while(empty_check == 0);
            rotate_rad = atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index+1].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index+1].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x));
        }
    }
    else if(panel_right_cnt == 6)
    {
        cout << "all panel right"<< endl;
        if(min_dist_point_index == 5 )
        {
            double empty_check = 0;
            do{
                empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
            }while(empty_check == 0);
            rotate_rad = 0.5 * PI - atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x));
        }
        else
        {
            cout << "all panel right and not in index 5"<< endl;
            double empty_check = 0;
            do{
                empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
            }while(empty_check == 0);
            rotate_rad = 0.5 * PI - atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index+1].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index+1].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x));
        }
    }
    else
    {
        if(min_dist_point_index == 5)
        {
            cout << "in index 5 on the right"<< endl;
            if( mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y < (0 ))
            {
                double empty_check = 0;
                do{
                    empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
                }while(empty_check == 0);
                rotate_rad = atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x));
            }
            else
            {
                cout << "in index 5 on the left"<< endl;
                double empty_check = 0;
                do{
                    empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
                }while(empty_check == 0);
                rotate_rad = 0.5 * PI - atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index-1].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index-1].x));
            }
        }
        else
        {
            if( mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y < (0 ) )
            {
                cout << "not in index 5 and which is on the right"<< endl;
                double empty_check = 0;
                do{
                    empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
                }while(empty_check == 0);
                rotate_rad = atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index+1].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index+1].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x));
            }

            else
            {
                cout << "not in index 5 and which is on the left"<< endl;
                double empty_check = 0;
                do{
                    empty_check = mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
                }while(empty_check == 0);
                rotate_rad = 0.5 * PI - atan(abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index-1].y) / abs(mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[min_dist_point_index-1].x));
            }
        }
    }

    // Get final parking heading value --------------------------------------------------------------------------------------------------------------
    //
    double panel_final_parking_vec_angle = 0;
    do{
        double panel_final_parking_vec_x = mpc_velodyne->GetPCL()->panelpoint_cloud->points[1].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].x;
        double panel_final_parking_vec_y = mpc_velodyne->GetPCL()->panelpoint_cloud->points[1].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
        panel_final_parking_vec_angle = atan2(panel_final_parking_vec_y,panel_final_parking_vec_x);
        if(panel_final_parking_vec_angle < 0)
        {
            panel_final_parking_vec_angle += 2*PI;
        }
//        cout << "can not find panel points" << endl;
    }while(panel_final_parking_vec_angle == 0);

//    cout << "panel angle : " << panel_final_parking_vec_angle << endl;

    double ugv_heading_vec_x = -1.0;
    double ugv_heading_vec_y = 0.0;
    double ugv_heading_angle = atan2(ugv_heading_vec_y, ugv_heading_vec_x);
    if(ugv_heading_angle < 0)
    {
        ugv_heading_angle += 2*PI;
    }

    double final_parking_and_ugv_heading_differ = panel_final_parking_vec_angle - PI;

    double current_ugv_heading = (mpc_velodyne->GetIMUData()).at(2);

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

//    cout <<"final parking heading : " << final_parking_heading << endl;
//    cout <<"current heading : " << current_ugv_heading << endl;
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

        rec_imu_data_tmp = mpc_velodyne->GetIMUData();
        cur_heading_degree_tmp = rec_imu_data_tmp.at(2) * 180 / PI;

        driving_struct.direction = UGV_move_right;
        driving_struct.velocity =100;

        mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);

        driving_struct.direction = UGV_move_right;
        driving_struct.velocity =0;

        cout << "turning for compensation" << endl;
        Sleep(500);
    }

    driving_struct.direction = UGV_move_backward;
    driving_struct.velocity =0;

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
    double side_center_margin = 1.0;
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
            memcpy(lrf_distance, &lrf_distance_raw[s_lrf_index], sizeof(long)*(number_of_point));
            mpc_velodyne->SetLRFDataToPCL(lrf_distance,number_of_point);

            if(!mpc_velodyne->GetLRFPanelFindStatus())
            {
                driving_struct.direction = UGV_move_left;
                driving_struct.velocity =100;
            }
            else
            {

                do{
                    detected_panel_info = mpc_velodyne->GetLRFPanelInfo();
                }while((detected_panel_info.at(0) == 0) &&(detected_panel_info.at(0) == 0));

                double panel_slope = detected_panel_info.at(1)/detected_panel_info.at(0);
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
//                    double panel_center_to_origin_x = -(detected_panel_info.at(3) - side_center_margin);

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

//                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_slope_norm_y - panel_center_to_origin_y*panel_slope_norm_x;


                    if(outer_product_panel_center_norm  > 0)
                    {
                        if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_left;
                            driving_struct.velocity =100;
                        }
                        else if ( ((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 360)))
                        {
                            driving_struct.direction = UGV_move_right;
                            driving_struct.velocity =100;
                        }
                        else
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity =100;
                        }
                    }
                    else
                    {
                        if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_differ_left;
                            driving_struct.velocity =100;
                        }
                        else if ((panel_slope_norm_angle > (PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                        {
                            driving_struct.direction = UGV_move_right;
                            driving_struct.velocity =100;
                        }
                        else
                        {
                            driving_struct.direction = UGV_move_forward;
                            driving_struct.velocity =100;
                        }
                    }
                }
                else if(panel_length > 2.0)
                {
                    driving_struct.direction = UGV_move_left;
                    driving_struct.velocity =100;
                }
                else// front or back
                {
                    double current_ugv_heading = -(mpc_velodyne->GetIMUData()).at(2);

                    if(current_ugv_heading < 0)
                    {
                        current_ugv_heading += 2*PI;
                    }


                    if( (current_ugv_heading > (final_parking_heading - 0.8)) && (current_ugv_heading < (final_parking_heading + 0.8)))
                    {
                        front_heading_range_satisfiled = true;
                        front_center_margin = 0.0;
                    }
                    else
                    {

                        front_heading_range_satisfiled = false;
                        front_center_margin = 0.8;
                    }

//                    if(front_heading_range_satisfiled == false)
//                    {
//                        front_center_margin = 0.8;
//                    }


//                    double panel_center_to_origin_x = -(detected_panel_info.at(3) - front_center_margin);
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



                    double front_panel_center_to_origin_x = - detected_panel_info.at(3);
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

                    double front_panel_center_x_to_ugv = sqrt(detected_panel_info.at(3)*detected_panel_info.at(3) + detected_panel_info.at(4)*detected_panel_info.at(4)) * sin(angle_between_norm_origin);


//                    double outer_product_panel_center_norm = panel_center_to_origin_x*panel_slope_norm_y - panel_center_to_origin_y*panel_slope_norm_x;


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

                            if((panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (1.5*PI - (5.0/180.0*PI))))
                            {
                                driving_struct.direction = UGV_move_left;
                                driving_struct.velocity =100;
                            }
                            else if (((panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI))) && (panel_slope_norm_angle >= 0)) || ((panel_slope_norm_angle > (1.5*PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < 360)))
                            {
                                driving_struct.direction = UGV_move_right;
                                driving_struct.velocity =100;
                            }
                            else
                            {
                                driving_struct.direction = UGV_move_forward;
                                if(front_heading_range_satisfiled == true)
                                {
                                    driving_struct.velocity =50;
                                }
                                else
                                {
                                    driving_struct.velocity =100;
                                }

                            }
                        }
                        else
                        {
                            if(front_heading_range_satisfiled == false)
                            {
                                if((panel_slope_norm_angle > (0.0 + (5.0/180.0*PI))) && (panel_slope_norm_angle < (PI - (5.0/180.0*PI))))
                                {
                                    driving_struct.direction = UGV_move_differ_left;
                                    driving_struct.velocity =100;
                                }
                                else if ((panel_slope_norm_angle > (PI + (5.0/180.0*PI))) && (panel_slope_norm_angle < (2*PI - (5.0/180.0*PI))))
                                {
                                    driving_struct.direction = UGV_move_right;
                                    driving_struct.velocity =100;
                                }
                                else
                                {
                                    driving_struct.direction = UGV_move_forward;
                                    driving_struct.velocity =100;
                                }
                            }
                            else
                            {
                                if(panel_slope_norm_angle > (0.5*PI + (5.0/180.0*PI)))
                                {
                                    driving_struct.direction = UGV_move_left;
                                    driving_struct.velocity =100;
                                }
                                else if (panel_slope_norm_angle < (0.5*PI - (5.0/180.0*PI)))
                                {
                                    driving_struct.direction = UGV_move_right;
                                    driving_struct.velocity =100;
                                }
                                else if (front_panel_center_x_to_ugv > 0.3)
                                {
                                    driving_struct.direction = UGV_move_backward;
                                    driving_struct.velocity =50;
                                }
                                else
                                {
                                    break;
                                }
//                                else if (outer_product_panel_center_norm < -0.05)
//                                {
//                                    driving_struct.direction = UGV_move_backward;
//                                    driving_struct.velocity =90;
//                                }
                            }
                        }
                    }
                }

            }

            cout << " norm : " << panel_slope_norm_x << " , "  << panel_slope_norm_y << " control :  " << driving_struct.direction<< endl;
            mpc_vehicle->Move(driving_struct.direction, driving_struct.velocity);


        }
    }while(driving_struct.driving_mission);


    cout << "Parking finished !" << endl;
    return true;
}

bool CDriving::AttitudeEstimation(){

    vector<double> rec_imu_data;

    double panel_front_vec_x,panel_front_vec_y;
    for(int i = 0; i < 10;i++)
    {
        while(!mpc_velodyne->IsPanelFound()) {}
        rec_imu_data = mpc_velodyne->GetIMUData();

        panel_front_vec_x = mpc_velodyne->GetPCL()->panelpoint_cloud->points[1].x - mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].x;
        panel_front_vec_y = mpc_velodyne->GetPCL()->panelpoint_cloud->points[1].y - mpc_velodyne->GetPCL()->panelpoint_cloud->points[0].y;
    }

//    double panel_front_vec_angle = atan2(panel_front_vec_y,panel_front_vec_x);

//    double ugv_heading_vec_x = -1.0;
//    double ugv_heading_vec_y = 0.0;
//    double ugv_heading_vec_angle = atan2(ugv_heading_vec_y, ugv_heading_vec_x);

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
