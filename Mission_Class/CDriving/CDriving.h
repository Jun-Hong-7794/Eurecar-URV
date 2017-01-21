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
#define DRIVE_INX_DRIVE_TO_PANEL 1
#define DRIVE_INX_PARKING__PANEL 2

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

    //Mutex
    QMutex mtx_driving_struct;
    QMutex mtx_parking_struct;

private:
    //-------------------------------------------------
    // Device Class
    //-------------------------------------------------
    CGPS* mpc_gps;
    CLRF* mpc_lrf;
    CCamera* mpc_camera;
    CKinova* mpc_kinova;
    CVehicle* mpc_vehicle;
    CVelodyne* mpc_velodyne;

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

    DRIVING_STRUCT GetDrivingOption();
    PARKING_STRUCT GetParkingOption();
    //-------------------------------------------------
    // Drive Class Main Function
    //-------------------------------------------------
    bool DriveToPanel();
    bool ParkingFrontPanel();

signals:
    void SignalVelodyneParser(bool _parser_complete);

};

#endif // CDRIVING_H













