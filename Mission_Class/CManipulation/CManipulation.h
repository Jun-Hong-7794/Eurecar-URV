#ifndef CMANIPULATION_H
#define CMANIPULATION_H

//-------------------------------------------------
// Qt Dependences Class
//-------------------------------------------------
#include <QThread>
#include <QMutex>

//-------------------------------------------------
// ElementTech Class
//-------------------------------------------------
#include "ElementTech_Class/CRGBD/CRGBD.h"
#include "Def_Manipulation.h"

//-------------------------------------------------
// Device Class
//-------------------------------------------------
#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"
#include "Device_Class/CKinova/CKinova.h"
#include "Device_Class/CVehicle/CVehicle.h"
#include "Device_Class/CVelodyne/CVelodyne.h"
#include "Device_Class/CGripper/CGripper.h"

//-------------------------------------------------
// Definetion
//-------------------------------------------------
#define MANIPUL_INX_LRF_KINOVA           1
#define MANIPUL_INX_KINOVA_FORCE_CLRL    2
#define MANIPUL_INX_GRIPPER_FORCE_CLRL   3

class CManipulation:public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CManipulation();
    CManipulation(CLRF* _p_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne, CGripper* _p_gripper);
    ~CManipulation();
private:
    int m_main_fnc_index;

    LRF_KINOVA_STRUCT mstruct_lrf_kinova;
    QMutex mxt_lrf_kinova;

    KINOVA_FORCE_CTRL_STRUCT mstruct_kinova_force_ctrl;
    QMutex mxt_kinova_force_ctrl;

    GRIPPER_FORCE_CTRL_STRUCT mstruct_gripper_force_ctrl;
    QMutex mxt_gripper_force_ctrl;
    //-------------------------------------------------
    // ElementTech Class
    //-------------------------------------------------
    CRGBD* mpc_rgb_d;

private:
    //-------------------------------------------------
    // Device Class
    //-------------------------------------------------
    CLRF* mpc_lrf;
    CCamera* mpc_camera;
    CKinova* mpc_kinova;
    CVehicle* mpc_vehicle;
    CVelodyne* mpc_velodyne;
    CGripper* mpc_gripper;

public:
    //-------------------------------------------------
    // Dvice Class Initialize(Connect) Function
    //-------------------------------------------------
    //KINOVA
    bool InitKinova();
    bool CloseKinova();
    bool KinovaInitMotion();
    bool KinovaAlignPanel();
    bool KinovaGetPosition(CartesianPosition& _position);
    bool KinovaDoManipulate(CartesianPosition _position);

    bool KinovaMoveUnitStepUp();
    bool KinovaMoveUnitStepDw();
    bool KinovaMoveUnitStepRi();
    bool KinovaMoveUnitStepLe();
    bool KinovaMoveUnitStepFw();
    bool KinovaMoveUnitStepBw();

    //LRF
    bool InitLRF();
    bool CloseLRF();
    bool IsLRFConnected();
    bool GetLRFInfo(double &_slope, double &_distance, double _s_deg, double _e_deg);

    //Camera
    bool InitCamera();
    void CloseCamera();
    bool IsCameraConnected();

    bool SetRGBDFunction(int _index);

    //End Effector
    bool InitGripper(char* _device_port = (char *)"/dev/ttyUSB0");
    bool CloseGripper();
    bool GripperGoRelPose(double _deg);
    bool GripperGoThePose(double _deg);
    bool GripperPresentPose(uint16_t& _pose);
    bool GripperPresentLoad(uint16_t& _load);


public:
    bool SelectMainFunction(int _fnc_index_);

    void SetManipulationOption(LRF_KINOVA_STRUCT _manipulation_option);
    LRF_KINOVA_STRUCT GetLRFKinovaOption();

    void SetManipulationOption(KINOVA_FORCE_CTRL_STRUCT _manipulation_option);
    KINOVA_FORCE_CTRL_STRUCT GetKinovaForceCtrlOption();

    void SetManipulationOption(GRIPPER_FORCE_CTRL_STRUCT _manipulation_option);
    GRIPPER_FORCE_CTRL_STRUCT GetGripperForceCtrlOption();

private:
    //-------------------------------------------------
    // Main Function
    //-------------------------------------------------
    bool LRFKinovaDepthControl();
    bool KinovaForceCtrl();
    bool GripperForceCtrl();

signals:
    void SignalKinovaPosition(CartesianPosition);
    void SignalKinovaForceVector(CartesianPosition);

    void SignalLRFImage(cv::Mat);
    void SignalCameraImage(cv::Mat);
    void SignalSegnetImage(cv::Mat);
};

#endif // CMANIPULATION_H
