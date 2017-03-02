#pragma once
#include <QtWidgets>
#include <QThread>

#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <dlfcn.h>
#include <stdlib.h>

#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <KinovaTypes.h>

#define KINOVA_PI 3.1415926535
#define KINOVA_D2R (3.141592/180)
#define KINOVA_R2D (180/3.141592)

#define VEL             0.09 //[cm/s]
#define ROT             0.5 //[cm/s]
#define STEP_NUM        5 // unit
#define SLEEP_TIME      5000

#define ALIGN_TO_PANEL_ROLL_VALUE 2.0974
#define ALIGN_TO_PANEL_PITCH_VALUE 1.5575
#define ALIGN_TO_PANEL_YAW_VALUE -2.2821

typedef enum Direction {CW, CCW} VALVE_ROTATE_DIR;

class CKinova: public QThread{
    Q_OBJECT
protected:
    void run();

public:
    CKinova();
    ~CKinova();

public:
    bool InitKinova();
    void CloseKinova();

    bool IsKinovaInitialized();
    bool IsKinovaInitPosition();

private:
    void * mp_commandLayer_handle;

    int m_kinova_devicesCount;

    bool fl_kinova_init;
    bool fl_kinova_init_position;
    bool fl_kinova_manipulation;

    bool fl_kinova_get_force_feedback;

    KinovaDevice m_kinova_list[MAX_KINOVA_DEVICE];

    double m_kinova_rel_pos;
    CartesianPosition m_kinova_disired_position;

public://Function pointers to the functions we need

    int (*Kinova_InitAPI)();
    int (*Kinova_CloseAPI)();

    int (*Kinova_RestoreFactoryDefault)();

    int (*Kinova_SetActiveDevice)(KinovaDevice device);
    int (*Kinova_GetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);

    int (*Kinova_SetTorqueCommandMax)(float Command[COMMAND_SIZE]);

    int (*Kinova_GetCartesianCommand)(CartesianPosition &);
    int (*Kinova_GetCartesianPosition)(CartesianPosition &);
    int (*Kinova_GetForcesInfo)(ForcesInfo &);
    int (*Kinova_GetJoystickValue)(JoystickCommand &);

    int (*Kinova_MoveHome)();

    int (*KinovaUnitStepMoving)(TrajectoryPoint command);
    //int (*Kinova_InitFingers)();
    int (*Kinova_SetFrameType)(int);

    //Arm & Finger
    int (*Kinova_SendBasicTrajectory)(TrajectoryPoint command);
    int (*Kinova_SendAdvanceTrajectory)(TrajectoryPoint command);
    int (*Kinova_SendJoystickCommand)(JoystickCommand command);
    int (*Kinova_EraseAllTrajectories)();

    int (*Kinova_GetQuickStatus)(QuickStatus &);
    int (*Kinova_GetCartesianForce)(CartesianPosition &);



private:
    double m_base_frame_rot;

private:
    double m_scan_unit_step;
    double m_scan_current_step;

    double m_scan_final_z;
    double m_scan_current_z;
    double m_scan_next_z;

    CartesianPosition m_cartisian_ref_position;

    /*---------------------------------------------*/
    double theta; // [deg]
    double x_coord;
    double y_coord;
    double z_coord;
    double initialAngle; // [deg]
    bool fl_using_current_coor;
    /*---------------------------------------------*/

public: // Basic Motion
    void KinovaInitMotion();
    void KinovaAlignToPanel();
    void KinovaResetFactorial();

    void KinovaRotateBase(double _rot_deg);

    bool KinovaDoManipulate(CartesianPosition _desired_position, int _mode = 1, double _force_threshold = 0);//mode = 1 : Joystic, mode = 2 : Trajectory
    bool KinovaDoManipulate(CartesianPosition _desired_position, double _force_threshold);
    bool KinovaForceCtrl();

    CartesianPosition KinovaGetPosition();
    CartesianPosition KinovaGetCartesianForce();

    void KinovaSetCartesianForceThread();
    CartesianPosition KinovaGetCartesianForceThread();

    bool KinovaMoveUnitStep(double _x, double _y, double _z, double _th_x = 0, double _th_y = 0, double _th_z = 0 );

    bool KinovaMoveUnitStepUp();
    bool KinovaMoveUnitStepDw();
    bool KinovaMoveUnitStepRi();
    bool KinovaMoveUnitStepLe();
    bool KinovaMoveUnitStepFw();
    bool KinovaMoveUnitStepBw();

    void SetKinovaRotateValve(bool _using_current_coord, bool _init_angle,double _x, double _y, double _z);
    void KinovaRotateValveMotion(VALVE_ROTATE_DIR _dir, int _radius, int _theta);

    bool KinovaRotateValveMotion(bool _using_constant_c_point, CartesianPosition _center_point, int _radius, int _theta, double _unit_angle = 5/*deg*/);

    bool Kinova_Do_Manipulate(JoystickCommand _desired_command);

    bool Kinova_Scan_Init(double _rel_pos,double unit_step/*m*/,CartesianPosition _ref_position);
    bool Kinova_Scan_Moving();//Up, Down Moving From Current Position to Relative Position
    void Kinova_Scan_End();

    bool Kinova_Scan_Moving(double _rel_pos,CartesianPosition _ref_position);//Up, Down Moving From Current Position to Relative Position
    bool Kinova_Do_Manipulate(TrajectoryPoint _desired_position,int _mode = 1);//mode = 1 : Joystic, mode = 2 : Trajectory

    double Kinova_Get_Rel_Pos_Z(CartesianPosition _ref_position);//Up, Down Moving From Current Position to Relative Position


    void Kinova_Do_Scan(double _kinova_rel_pos, CartesianPosition _desired_position);

    void Kinova_Do_Wrench_Scan(double _kinova_rel_pos);

public:// Complex Motion


    void Kinova_GoTo_Wrench();
    void Kinova_GoTo_Valve();

    void Kinova_Ready_To_Grasp();
    void Kinova_Ready_To_Rotate();
    void Kinova_Wrench_Injection();
public:
    bool IsScanning();

signals:
    void Get_Kinova_Position(CartesianPosition _position);
    void SignalKinovaPosition(CartesianPosition _position);
    void SignalKinovaForce(CartesianPosition _position);
};
