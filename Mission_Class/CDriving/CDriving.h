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
#include "Device_Class/CGPS/CGPS.h"
#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"
#include "Device_Class/CKinova/CKinova.h"
#include "Device_Class/CVehicle/CVehicle.h"
#include "Device_Class/CVelodyne/CVelodyne.h"

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

class CDriving:public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CDriving();
    CDriving(CGPS* _p_gps, CLRF* _p_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne);

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

private:
    //-------------------------------------------------
    // Device Class
    //-------------------------------------------------
    CGPS* mpc_gps;
    CLRF* mpc_drive_lrf;
    CCamera* mpc_camera;
    CKinova* mpc_kinova;
    CVehicle* mpc_vehicle;
    CVelodyne* mpc_velodyne;
    CRGBD* mpc_rgb_d;

public:

    //-------------------------------------------------
    // Dvice Class Initialize(Connect) Function
    //-------------------------------------------------
    bool ConnectVehicle();

    bool ConnectVelodyne();
    bool CloseVelodyne();
    bool IsVelodyneConnected();

    void PCLInit();
    CPCL* GetPCL();


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
    bool DriveToPanel();
    bool  ParkingFrontPanel();
    bool LRFVehicleHorizenControl();
    bool LRFVehicleAngleControl();

    int GetPanelHeadingError();

signals:
    void SignalVelodyneParser(bool _parser_complete);

    //View
    void SignalLRFMapImage(cv::Mat);
    //Display
    void SignalLRFVehicleAngleStruct(LRF_VEHICLE_ANGLE_STRUCT);
    void SignalLRFVehicleHorizenStruct(LRF_VEHICLE_HORIZEN_STRUCT);
};

#endif // CDRIVING_H













