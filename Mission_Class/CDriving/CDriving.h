#ifndef CDRIVING_H
#define CDRIVING_H

//-------------------------------------------------
// Qt Dependences Class
//-------------------------------------------------
#include <QThread>
#include <QMutex>

//-------------------------------------------------
// Device Class
//-------------------------------------------------
#include "Device_Class/CIMU/CIMU.h"
#include "Device_Class/CGPS/CGPS.h"
#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"
#include "Device_Class/CKinova/CKinova.h"
#include "Device_Class/CVehicle/CVehicle.h"
#include "Device_Class/CVelodyne/CVelodyne.h"
#include "Device_Class/CLMS511/CLMS511.h"

//-------------------------------------------------
// Element Tech Class
//-------------------------------------------------
#include "ElementTech_Class/CNavigation/CNavigation.h"
#include "ElementTech_Class/CRGBD/CRGBD.h"

//-------------------------------------------------
// Driving Definetion
//-------------------------------------------------
#include "Mission_Class/CDriving/Def_Driving.h"

//-------------------------------------------------
// Definetion
//-------------------------------------------------
#define DRIVE_INX_DRIVE_TO_PANEL       1
#define DRIVE_INX_PARKING__PANEL       2
#define DRIVE_INX_LRF_VEHICLE_ANGLE    3
#define DRIVE_INX_LRF_VEHICLE_HORIZEN  4

#define MAX_VEL 200
#define MIN_VEL 80
#define ACCEL_RATE 5
#define DECEL_RATE -8
#define DECEL_START_DIST 1.8

class CDriving:public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CDriving();
    CDriving(CIMU* _p_imu, CGPS* _p_gps, CLRF* _p_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne, CLMS511* _p_lms511);

    ~CDriving();
private:
    int m_main_fnc_index; // 1: DriveTopanel(), 2: ParkingFrontPanel()

    DRIVING_STRUCT mstruct_driving;
    PARKING_STRUCT mstruct_parking;
    LRF_VEHICLE_ANGLE_STRUCT mstruct_lrf_vehicle_angle;
    LRF_VEHICLE_HORIZEN_STRUCT mstruct_lrf_vehicle;

    //Mutex
    QMutex mtx_driving_struct;
    QMutex mtx_parking_struct;
    QMutex mxt_lrf_vehicle;
    QMutex mxt_lrf_vehicle_angle;


    // Aerna info
    vector<vector<double>> m_arena_info = {{0,-45},{-60,-45},{-60,45},{0,45}};
    bool arena_aligned_compensate = false;
    bool IMU_update_finished = true;

    // Aligned initial heading
    double aligned_initial_heading = 0.0;
    double aligned_heading_to_current_differ = 0.0;
    vector<int> past_encoder_value = {0,0};
    vector<int> current_encoder_value = {0,0};
    bool encoder_bias_update = false;
    long int encoder_bias_1 = 0;
    long int encoder_bias_2 = 0;
    vector<double> arena_move_coordinate = {0,0};
    vector<double> arena_move_coordinate_old = {0,0};
    double total_move_distance = 0;



    // parking parameter
    double side_center_margin = 0.9;
    double desirable_parking_dist = 0.9;

    // ugv info variables
    double ugv_heading = 0.0;
    double final_parking_heading = -PI;

    double panel_slope_norm_x = 0.0;
    double panel_slope_norm_y = 0.0;
    double panel_way_x = 0.0;
    double panel_way_y = 0.0;
    bool parking_short = true;
    bool panel_found = true;

    double m_dist_error_from_p1_km=0.0;

    vector<double> euler_angles = {0,0,0};
    mip_filter_linear_acceleration linear_accel;
    mip_ahrs_internal_timestamp time_stamp;
    mip_ahrs_delta_velocity delta_velocity;




private:
    //-------------------------------------------------
    // Device Class
    //-------------------------------------------------
    CIMU* mpc_imu;
    CGPS* mpc_gps;
    CLRF* mpc_drive_lrf;
    CCamera* mpc_camera;
    CKinova* mpc_kinova;
    CVehicle* mpc_vehicle;
    CVelodyne* mpc_velodyne;
    CRGBD* mpc_rgb_d;
    CLMS511* mpc_lms511;


public:

    //-------------------------------------------------
    // Dvice Class Initialize(Connect) Function
    //-------------------------------------------------
    bool ConnectVehicle();

    bool ConnectVelodyne();
    bool CloseVelodyne();
    bool IsVelodyneConnected();

    bool IsGPSConnected();
    bool CloseGPS();
    void SetInitGPSpoint();
    void SetGroundGPS();

    void PCLInit();
    CPCL* GetPCL();
    CGPS* GetGPS();

    vector<double> GetIMUEuler();
    mip_filter_linear_acceleration GetIMULinearAccel();
    mip_ahrs_internal_timestamp GetIMUTimestamp();
    mip_ahrs_delta_velocity GetIMUDeltaVelocity();
    CVelodyne* GetVelodyne();
    int GetPanelHeadingError();
    vector<double> GetParkingStatus();
    vector<int> GetEncoderValueRaw();


    void SetArenaInfo(vector<double> _lb, vector<double> _lt, vector<double> _rt, vector<double> _rb);
    void SetArenaShift(bool _shift_on);
    void SetPanelDistance(double _panel_dist_dri, double _panel_dist_par);

    //-------------------------------------------------
    // Drive Class Option Function
    //-------------------------------------------------
    bool SelectMainFunction(int _fnc_index_);

    void SetDrivingOption(DRIVING_STRUCT _driving_option);
    void SetParkingOption(PARKING_STRUCT _driving_option);
    void SetManipulationOption(LRF_VEHICLE_ANGLE_STRUCT _driving_option);
    void SetManipulationOption(LRF_VEHICLE_HORIZEN_STRUCT _driving_option);

    vector<double> GetWaypointError(double _way_x,double _way_y);

    int GetParkingControl(vector<double> _waypoint_error);


    DRIVING_STRUCT GetDrivingOption();
    PARKING_STRUCT GetParkingOption();

    LRF_VEHICLE_ANGLE_STRUCT GetLRFVehicleAngleOption();
    LRF_VEHICLE_HORIZEN_STRUCT GetLRFVehicleHorizenOption();

    //-------------------------------------------------
    // Drive Class Main Function
    //-------------------------------------------------
    bool DrivingMissionManager();

    bool DriveToPanel();
    bool ParkingFrontPanel();
    int VelGen(double);

    bool LRFVehicleHorizenControl();
    bool LRFVehicleAngleControl();

    bool DrivieByOdometer(double _heading_constraint, double _distance_constraint);

    void PanelFrontDistanceControlByLMS511();

    double CalcDistErrorToCheckPoint(double _dist_m);



signals:
    void SignalVelodyneParser(bool _parser_complete);

    //View
    void SignalLRFMapImage(cv::Mat);
    //Display
    void SignalLRFVehicleAngleStruct(LRF_VEHICLE_ANGLE_STRUCT);
    void SignalLRFVehicleHorizenStruct(LRF_VEHICLE_HORIZEN_STRUCT);

    //Arena information query singal
    void SignalDrivingQueryArenaInfo();


public slots:
    void SlotIMUEuler(vector<double> _euler_angles);
    void SlotIMULinearAccel(mip_filter_linear_acceleration _linear_accel);
    void SlotIMUTimestamp(mip_ahrs_internal_timestamp _time_stamp);
    void SlotIMUDeltaVelocity(mip_ahrs_delta_velocity _delta_velocity);
    void SlotVelodynePanelFound(bool _panel_found);

    void SlotLMS511UpdatePoints(vector<vector<double>> _point_list);

};

#endif // CDRIVING_H













