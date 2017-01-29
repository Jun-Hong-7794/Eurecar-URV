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
#include "../CDriving/CDriving.h"
#include "../CManipulation/Def_Manipulation.h"

typedef struct _Step_Information{

    QString step_title;

}STEP_INFO;

typedef struct _Mission_Script{

    QString mission_title;

    std::vector<STEP_INFO> step_vecor;

}MISSION_SCRIPT;

typedef struct _Scenario_Script{

    QString scenario_title;
    QString mission_file_directory;

    std::vector<QString> mission_file_name;

    int number_of_mission;

}SCENARIO_SCRIPT;


#endif // DEF_SCRIPT_H
