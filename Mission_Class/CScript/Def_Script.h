#ifndef DEF_SCRIPT_H
#define DEF_SCRIPT_H

#include <QString>

//-------------------------------------------------
// Standard
//-------------------------------------------------
#include <vector>
#include <iostream>

//-------------------------------------------------
// Define Header
//-------------------------------------------------
#include "../CDriving/Def_Driving.h"
#include "../CManipulation/Def_Manipulation.h"

//-------------------------------------------------
// Function Index Define
//-------------------------------------------------
//Driving
#define DR_VELODYNE_VEHICLE_CONTROL          0x1000
#define DR_LRF_VEHICLE_ANGLE_CONTROL         0x1001
#define DR_LRF_VEHICLE_HORIZEN_CONTROL       0x1002
#define DR_VEHICLE_DRIVE_TO_PANEL            0x1003
#define DR_VEHICLE_PARKING                   0x1004
#define DR_PARKING_RETRY                     0x1005
#define DR_LOCALIZATION                      0x1006
#define DR_WRENCH_RECOG                      0x1007

//Manipulation
/*New LRF Kinova Control*/
#define MP_LRF_K_VERTIVAL_CONTROL            0x2017
#define MP_LRF_K_HORIZEN_CONTROL             0x2018
#define MP_LRF_K_ANGLE_CONTROL               0x2019

#define MP_LRF_V_HORIZEN_CONTROL             0x2020
#define MP_LRF_V_ANGLE_CONTROL               0x2021

#define MP_KINOVA_FORCE_CONTROL              0x2001
#define MP_KINOVA_FORCE_CHECK                0x2002

#define MP_GRIPPER_FORCE_CONTROL             0x2003
#define MP_GRIPPER_MAGNET_CONTROL            0x2004

#define MP_GRIPPER_VALVE_SIZE_RECOG          0x2005

#define MP_KINOVA_MANIPULATE                 0x2006
#define MP_KINOVA_ROTATE_VALVE               0x2007

#define MP_GRIPPER_GO_TO_REL_POSE            0x2022

#define MP_ROTATOR                           0x2023

#define MP_KINOVA_LRF_VALVE_SEARCHING        0x2024

#define MP_KINOVA_LRF_CHECK_V_DST            0x2025

#define MP_GRIPPER_FIND_VALVE_LOCATION       0x2026

#define MP_KINOVA_CURRENT_POSITION           0x2027
#define MP_KINOVA_WRENCH_SEARCHING           0x2028
/*Old LRF Kinova Control*/
#define MP_LRF_KINOVA_VERTIVAL_CONTROL       0x2008
#define MP_LRF_KINOVA_HORIZEN_CONTROL        0x2009
#define MP_LRF_KINOVA_ANGLE_CONTROL          0x2010

#define MP_WRENCH_RECOGNITION                0x2011

#define MP_LRF_KINOVA_WRENCH_LOVATION_CTRL   0x2012

#define MP_LRF_K_VEHICLE_HORIZEN_CONTROL     0x2013
#define MP_LRF_K_VEHICLE_ANGLE_CONTROL       0x2014

#define MP_KINOVA_ALIGN_TO_PANEL             0x2015
#define MP_KINOVA_FIT_TO_VALVE_POSE          0x2016

//Grammar
#define GR_CONDITIONALLY_ITERATION           0x3000
//-------------------------------------------------
// Script Variable Define
//-------------------------------------------------
typedef struct _Script_Double{

    double double_value;
    QString variable_name;

}SCRIPT_DOUBLE;

typedef struct _Script_Int{

    int int_value;
    QString variable_name;

}SCRIPT_INT;

typedef struct _Script_BOOL{

    bool bool_value;
    QString variable_name;

}SCRIPT_BOOL;

typedef struct _Conditionally_Iterate_Option{

    bool fl_condition;
    QString str_condition;

    int start_step_index;
    int end_step_index;

    QString str_mission_name; //Current Or File name( ex) Wrench Grasp, Inject Valve . . .

}CONDITIONALLY_ITERATE_OPTION;


typedef struct _Step_Information{

    QString step_title;

    bool fl_do_init_motion;
    int function_index; //0x1xxx : Driving, 0x2xxx : Manipulation

    DRIVING_OPTION driving_option;

    MANIPULATION_OPTION manipulation_option;

    CONDITIONALLY_ITERATE_OPTION iterate_option;

    int before_sleep; //Before Step Start
    int after__sleep; //After Step End

    QString str_if;
    bool fl_condition_if_flag;

    QString str_else;
    bool fl_condition_else_flag;

}STEP_INFO;

typedef struct _Mission_Script{

    QString mission_title;

    std::vector<STEP_INFO> step_vecor;

    std::vector<SCRIPT_INT> vec_lc_int; // Local int
    std::vector<SCRIPT_BOOL> vec_lc_bool; // Local bool
    std::vector<SCRIPT_DOUBLE> vec_lc_double; // // Local double

}MISSION_SCRIPT;

typedef struct _Scenario_Script{

    QString scenario_title;
    QString mission_file_directory;

    std::vector<QString> mission_file_name;
    std::vector<QString> mission_file_path;

    int number_of_mission;

}SCENARIO_SCRIPT;

typedef struct _Script_Player_Option{

    bool partial_play;

    int start_mission_num;
    int end_mission_num;

    int start_step_num;
    int end_step_num;

}SCRIPT_PLAYER_OPTION;


#endif // DEF_SCRIPT_H
