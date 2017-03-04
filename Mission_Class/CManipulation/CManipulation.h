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

/*New LRF Kinova Control*/
#define MANIPUL_INX_LRF_K_VERTIVAL_CTRL    19
#define MANIPUL_INX_LRF_K_HORIZEN_CTRL     20
#define MANIPUL_INX_LRF_K_ANGLE_CTRL       21

#define MANIPUL_INX_LRF_V_HORIZEN_CTRL     22
#define MANIPUL_INX_LRF_V_ANGLE_CTRL       23

/*Old LRF Kinova Control*/
#define MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL    1
#define MANIPUL_INX_LRF_KINOVA_HORIZEN_CTRL     2
#define MANIPUL_INX_LRF_KINOVA_ANGLE_CTRL       3

#define MANIPUL_INX_KINOVA_FORCE_CLRL           4
#define MANIPUL_INX_KINOVA_FORCE_CHECK          5
#define MANIPUL_INX_GRIPPER_FORCE_CLRL          6
#define MANIPUL_INX_GRIPPER_MAGNET_CLRL         7
#define MANIPUL_INX_GRIPPER_VALVE_SIZE_RECOG    8
#define MANIPUL_INX_KINOVA_MANIPULATE           9
#define MANIPUL_INX_KINOVA_ROTATE_VALVE         10
#define MANIPUL_INX_WRENCH_RECOGNITION          11
#define MANIPUL_INX_LRF_KINOVA_WRENCH_LOCATION  12

#define MANIPUL_INX_LRF_K_VEHICLE_HORIZEN_CTRL  13
#define MANIPUL_INX_LRF_K_VEHICLE_ANGLE_CTRL    14

#define MANIPUL_INX_KINOVA_ALIGN_TO_PANEL       15
#define MANIPUL_INX_KINOVA_INIT_MOTION          30
#define MANIPUL_INX_KINOVA_FIT_TO_VALVE         16

#define SENSING_LRF_PANEL_LOCALIZATION          17

#define MANIPUL_INX_GRIPPER_GO_TO_REL_POSE      18

#define MANIPUL_INX_ROTATOR                     24

#define MANIPUL_INX_KINOVA_LRF_VALVE_SEARCHING  25
#define MANIPUL_INX_KINOVA_LRF_CHECK_V_DST      26

class CManipulation:public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CManipulation();
    CManipulation(CLRF* _p_mani_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne, CGripper* _p_gripper, CSSD* _ssd);
    ~CManipulation();
private:
    int m_main_fnc_index;
    int mary_valve_size[6] = {16,17,18,19,22,24};

    int m_valve_size_graph_index;

    int m_valve_size_result;
    double m_valve_rotation_result;

    bool fl_main_fnc_result;
    bool fl_kinova_force_ctrl_result;

    int m_valve_size;

    int m_kinova_valve_pose_right;
    int m_kinova_valve_pose_left;

    int m_valve_size_retry_num;

    cv::Mat m_mat_panel_model;
    QMutex mtx_panel_model;

    //---------------Rotator---------------//

    ROTATOR_STRUCT mstruct_rotator;
    QMutex mxt_rotator;

    //---------------Kinova---------------//

    LRF_K_V_CTRL_STRUCT mstruct_lrf_k_v_ctrl;
    QMutex mxt_lrf_k_v_ctrl;

    LRF_K_H_CTRL_STRUCT mstruct_lrf_k_h_ctrl;
    QMutex mxt_lrf_k_h_ctrl;

    LRF_K_A_CTRL_STRUCT mstruct_lrf_k_a_ctrl;
    QMutex mxt_lrf_k_a_ctrl;

    //---------------Vehicle---------------//
    LRF_V_H_CTRL_STRUCT mstruct_lrf_v_h_ctrl;
    QMutex mxt_lrf_v_h_ctrl;

    LRF_V_A_CTRL_STRUCT mstruct_lrf_v_a_ctrl;
    QMutex mxt_lrf_v_a_ctrl;
    ////////////////////////////////////////

    LRF_SENSING_INFO_STRUCT mstruct_lrf_sensing_info;
    QMutex mxt_lrf_sensing_info;

    LRF_KINOVA_VERTICAL_CTRL_STRUCT mstruct_lrf_kinova_vertical;
    QMutex mxt_lrf_kinova_vertical;

    LRF_KINOVA_HORIZEN_CTRL_STRUCT mstruct_lrf_kinova_horizen;
    QMutex mxt_lrf_kinova_horizen;

    LRF_K_VEHICLE_ANGLE_STRUCT mstruct_lrf_k_vehicle_angle;
    QMutex mxt_lrf_k_vehicle_angle;

    LRF_K_VEHICLE_HORIZEN_STRUCT mstruct_lrf_k_vehicle_horizen;
    QMutex mxt_lrf_k_vehicle_horizen;

    LRF_KINOVA_WRENCH_LOCATION_STRUCT mstruct_lrf_kinova_wrench;
    QMutex mxt_lrf_kinova_wrench;

    KINOVA_FORCE_CTRL_STRUCT mstruct_kinova_force_ctrl;
    QMutex mxt_kinova_force_ctrl;

    KINOVA_FORCE_CHECK_STRUCT mstruct_kinova_force_check;
    QMutex mxt_kinova_force_check;

    GRIPPER_FORCE_CTRL_STRUCT mstruct_gripper_force_ctrl;
    QMutex mxt_gripper_force_ctrl;

    GRIPPER_GO_TO_REL_POSE_STRUCT mstruct_gripper_go_to_rel_pose;
    QMutex mxt_gripper_go_to_rel_pose;

    GRIPPER_MAGNET_CTRL_STRUCT mstruct_gripper_magnet_ctrl;
    QMutex mxt_gripper_magnet_ctrl;

    KINOVA_DO_MANIPULATE_STRUCT mstruct_kinova_manipulate;
    QMutex mxt_kinova_manipulate;

    KINOVA_ROTATE_VALVE_STRUCT mstruct_kinova_rotate_valve;
    QMutex mxt_kinova_rotate_valve;

    GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT mstruct_gripper_kinova_valve_recog;
    QMutex mxt_gripper_kinova_valve_recog;

    WRENCH_RECOGNITION mstruct_wrench_recognition;
    QMutex mxt_wrench_recognition;

    KINOVA_FIT_TO_VALVE_POSE_STRUCT mstruct_fit_to_valve_pose;
    QMutex mxt_fit_to_valve_pose;

    KINOVA_LRF_VALVE_SEARCHING_STRUCT mstruct_valve_searching;
    QMutex mxt_valve_searching;

    CHECK_CURRENT_V_DISTANCE_STRUCT mstruct_check_v_dst;
    QMutex mxt__check_v_dst;
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
    CSSD* mpc_ssd;

    //-------------------------------------------------
    // Valve Model Vector
    //-------------------------------------------------
    QVector<double> mqvec_16mm;
    QVector<double> mqvec_17mm;
    QVector<double> mqvec_18mm;
    QVector<double> mqvec_19mm;
    QVector<double> mqvec_22mm;
    QVector<double> mqvec_24mm;

    QVector<double> mqvec_sorted_16mm;
    QVector<double> mqvec_sorted_17mm;
    QVector<double> mqvec_sorted_18mm;
    QVector<double> mqvec_sorted_19mm;
    QVector<double> mqvec_sorted_22mm;
    QVector<double> mqvec_sorted_24mm;

public:
    //-------------------------------------------------
    // Calculate Function
    //-------------------------------------------------
    QVector<double> DataSort(QVector<double> _data);
    void DataSort(std::vector<GRIPPER_DATA>& _data);

    void ValveSizeDataModelInit(QVector<double> _data, int _index/*mm*/);

    int DataAnalisys(QVector<double> _data);//For Valve Recognition, Result: 16 ~ 24(16mm, 17mm, 18mm, 19mm, 22mm, 24mm)

    cv::Mat ValveModeling(int _valve_size, double _rotation_angle);

    void MakePanelModel(int _valve_size, int _wrench_index);
    cv::Mat GetPanelModel();
    void PanelModeling(int _valve_size, int _virtical_dst/*mm*/, int _horizen_dst/*mm*/, double _angle/*deg*/);
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

    bool KinovaMoveUnitStep(double _x, double _y, double _z, double _th_x, double _th_y, double _th_z);

    bool KinovaMoveUnitStepUp();
    bool KinovaMoveUnitStepDw();
    bool KinovaMoveUnitStepRi();
    bool KinovaMoveUnitStepLe();
    bool KinovaMoveUnitStepFw();
    bool KinovaMoveUnitStepBw();

    bool KinovaRotateBase(double _rot_deg);

    bool KinovaForceCheck(double _force_x, double _force_y, double _force_z); // true: force occured!

    //LRF
    bool InitLRF(char* _dev_path = (char *)"/dev/ttyACM0", int _dev_type = UST_20LX);
    bool CloseLRF();
    bool IsLRFConnected();
    bool GetLRFInfo(double &_slope, double &_distance, double _s_deg, double _e_deg);
    bool GetHorizenDistance(double _inlier_distance,double& _horizen_distance, double& _s_inlier_deg, double& _e_inlier_deg,
                                   double& _virt_s_deg, double& _virt_e_deg, double _s_deg, double _e_deg, int _sampling_loop);

    bool LocalizationOnPanel(int _mode, double _s_deg = 10/*deg*/, double _e_deg = 170/*deg*/,
                             int _inlier_dst = 1100/*mm*/, int _current_v_dst = 270/*mm, for const mode*/, double _current_ang = 0/*deg for const mode*/);
    //Camera
    bool InitCamera();
    void CloseCamera();
    bool IsCameraConnected();

    bool SetRGBDFunction(int _index);

    //End Effector
    bool InitGripper(char* _device_port = (char *)"/dev/ttyUSB0");
    bool CloseGripper();

    bool GripperGoThePose(int _pose_1, int _pose_2, int _load_thresh);

    bool GripperPresentPose(uint16_t& _pose);
    bool GripperPresentLoad(uint16_t& _load);

    //Rotator
    bool InitRotator(char* _device_port = (char *)"/dev/ttyUSB0");
    bool CloseRotator();

    bool RotatorGoThePose(int _step);

public:
    bool GetMainFunctionResult();
    void SetMainFunctionResult(bool _result);

    int GetValveSizeRecogResult();
    void SetValveSizeRecogResult(int _result);

    double GetValveRotationRecogResult();
    void SetValveRotationRecogResult(double _result);

    void SetForceCheckThread(bool _data);

public:
    bool SelectMainFunction(int _fnc_index_);

    //-------------------------------------------------
    // Sensor Test Function
    //-------------------------------------------------
    void SetManipulationOption(LRF_SENSING_INFO_STRUCT _manipulation_option);
    LRF_SENSING_INFO_STRUCT GetLRFSensingInfo();

    //---------------Rotator---------------//
    void SetManipulationOption(ROTATOR_STRUCT _manipulation_option);
    ROTATOR_STRUCT GetRotatorOption();

    //Main Function//
    /*LRF Ctrl New Version*/
    //---------------Kinova---------------//
    void SetManipulationOption(LRF_K_V_CTRL_STRUCT _manipulation_option);
    LRF_K_V_CTRL_STRUCT GetLRFKVerticalCtrlOption();

    void SetManipulationOption(LRF_K_H_CTRL_STRUCT _manipulation_option);
    LRF_K_H_CTRL_STRUCT GetLRFKHorizenCtrlOption();

    void SetManipulationOption(LRF_K_A_CTRL_STRUCT _manipulation_option);
    LRF_K_A_CTRL_STRUCT GetLRFKAngleCtrlOption();

    //---------------Vehicle---------------//
    void SetManipulationOption(LRF_V_A_CTRL_STRUCT _manipulation_option);
    LRF_V_A_CTRL_STRUCT GetLRFVAngleCtrlOption();

    void SetManipulationOption(LRF_V_H_CTRL_STRUCT _manipulation_option);
    LRF_V_H_CTRL_STRUCT GetLRFVHorizenCtrlOption();
    /////////////////////////////////////////

    void SetManipulationOption(LRF_KINOVA_VERTICAL_CTRL_STRUCT _manipulation_option);
    LRF_KINOVA_VERTICAL_CTRL_STRUCT GetLRFKinovaVerticalOption();

    void SetManipulationOption(LRF_KINOVA_HORIZEN_CTRL_STRUCT _manipulation_option);
    LRF_KINOVA_HORIZEN_CTRL_STRUCT GetLRFKinovaHorizenOption();

    void SetManipulationOption(LRF_K_VEHICLE_ANGLE_STRUCT _manipulation_option);
    LRF_K_VEHICLE_ANGLE_STRUCT GetLRFVehicleAngleOption();

    void SetManipulationOption(LRF_K_VEHICLE_HORIZEN_STRUCT _manipulation_option);
    LRF_K_VEHICLE_HORIZEN_STRUCT GetLRFVehicleHorizenOption();

    void SetManipulationOption(LRF_KINOVA_WRENCH_LOCATION_STRUCT _manipulation_option);
    LRF_KINOVA_WRENCH_LOCATION_STRUCT GetLRFKinovaWrenchLocationOption();

    void SetManipulationOption(KINOVA_FORCE_CTRL_STRUCT _manipulation_option);
    KINOVA_FORCE_CTRL_STRUCT GetKinovaForceCtrlOption();

    void SetManipulationOption(KINOVA_FORCE_CHECK_STRUCT _manipulation_option);
    KINOVA_FORCE_CHECK_STRUCT GetKinovaForceCheckOption();

    void SetManipulationOption(GRIPPER_FORCE_CTRL_STRUCT _manipulation_option);
    GRIPPER_FORCE_CTRL_STRUCT GetGripperForceCtrlOption();

    void SetManipulationOption(GRIPPER_GO_TO_REL_POSE_STRUCT _manipulation_option);
    GRIPPER_GO_TO_REL_POSE_STRUCT GetGripperGoToRelPoseOption();

    void SetManipulationOption(GRIPPER_MAGNET_CTRL_STRUCT _manipulation_option);
    GRIPPER_MAGNET_CTRL_STRUCT GetGripperMagnetCtrlOption();

    void SetManipulationOption(KINOVA_DO_MANIPULATE_STRUCT _manipulation_option);
    KINOVA_DO_MANIPULATE_STRUCT GetKinovaManipulateOption();

    void SetManipulationOption(KINOVA_ROTATE_VALVE_STRUCT _manipulation_option);
    KINOVA_ROTATE_VALVE_STRUCT GetKinovaRotateValveOption();

    void SetManipulationOption(GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT _manipulation_option);
    GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT GetGripperKinovaValveRecogOption();

    void SetManipulationOption(WRENCH_RECOGNITION _manipulation_option);
    WRENCH_RECOGNITION GetWrenchRecognitionOption();

    void SetManipulationOption(KINOVA_FIT_TO_VALVE_POSE_STRUCT _manipulation_option);
    KINOVA_FIT_TO_VALVE_POSE_STRUCT GetFitToValvePoseOption();

    void SetManipulationOption(KINOVA_LRF_VALVE_SEARCHING_STRUCT _manipulation_option);
    KINOVA_LRF_VALVE_SEARCHING_STRUCT GetKinovaLRFValveSearchingOption();

    void SetManipulationOption(CHECK_CURRENT_V_DISTANCE_STRUCT _manipulation_option);
    CHECK_CURRENT_V_DISTANCE_STRUCT GetCheckVDstOption();

private:
    //-------------------------------------------------
    // Dynamixel Pro Function
    //-------------------------------------------------
    bool DynamixelRotate();
    //-------------------------------------------------
    // Sensor Test Function
    //-------------------------------------------------
    bool LRFLocalizationOnPanel();

    //-------------------------------------------------
    // Main Function
    //-------------------------------------------------

    /*LRF Ctrl Old Version*/
    bool LRFKinovaVerticalControl();
    bool LRFKinovaHorizenControl();
    bool LRFKinovaWrenchLocationMove();

    bool LRFVehicleAngleControl();
    bool LRFVehicleHorizenControl();

    /*LRF Ctrl New Version*/

    //LRFK_XXX=> Kinova
    //LRFV_XXX=> Vehicle

    //XXX_VControl => Vertical Control
    //XXX_HControl => Horizen  Control
    //XXX_AControl => Angle    Control

    bool LRFK_VCtrl();
    bool LRFK_HCtrl();
    bool LRFK_ACtrl();
    bool LRFK_VCheck();

    bool LRFV_ACtrl();
    bool LRFV_HCtrl();
    /*----------------------------*/

    bool KinovaForceCtrl();
    bool KinovaForceCheck();
    bool KinovaDoManipulate();
    bool KinovaRotateValveMotion();

    bool KinovaLRFValveSearching();

    bool KinovaFitToValvePose();

    bool GripperKinovaValveSizeRecognition();
    bool GripperGoToRelPose();

    bool GripperForceCtrl();
    bool GripperMagnetCtrl();

    bool WrenchRecognition();

signals:
    void SignalKinovaPosition(CartesianPosition);
    void SignalKinovaForceVector(CartesianPosition);
    void SignalKinovaForceCheckOption(KINOVA_FORCE_CHECK_STRUCT);

    void SignalLRFKinovaAngleStruct(LRF_KINOVA_ANGLE_CTRL_STRUCT);
    void SignalLRFKinovaHorizenStruct(LRF_KINOVA_HORIZEN_CTRL_STRUCT);
    void SignalLRFKinovaVerticalStruct(LRF_KINOVA_VERTICAL_CTRL_STRUCT);

    void SignalValveSizeData(QVector<double> _x, QVector<double> _y, int _graph_index);

    void SignalLRFImage(cv::Mat);
    void SignalCameraImage(cv::Mat);
    void SignalSegnetImage(cv::Mat);

    void SignalValveImage(cv::Mat);
    void SignalPanelImage(cv::Mat);

    void SignalEditeGripperStatus(GRIPPER_STATUS);
};

#endif // CMANIPULATION_H
