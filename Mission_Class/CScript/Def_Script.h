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
#define MP_KINOVA_FORCE_CONTROL              0x2001
#define MP_KINOVA_FORCE_CHECK                0x2002
#define MP_GRIPPER_FORCE_CONTROL             0x2003
#define MP_GRIPPER_MAGNET_CONTROL            0x2004
#define MP_KINOVA_MANIPULATE                 0x2005
#define MP_KINOVA_ROTATE_VALVE               0x2006


#define MP_LRF_KINOVA_VERTIVAL_CONTROL       0x2006
#define MP_LRF_KINOVA_HORIZEN_CONTROL        0x2007
#define MP_LRF_KINOVA_ANGLE_CONTROL          0x2008

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

typedef struct _Step_Information{

    QString step_title;

    int function_index; //0x1xxx : Driving, 0x2xxx : Manipulation

    DRIVING_OPTION driving_option;

    MANIPULATION_OPTION manipulation_option;

    int before_sleep; //Before Step Start
    int after__sleep; //After Step End

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
