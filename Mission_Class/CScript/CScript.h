#ifndef CSCRIPT_H
#define CSCRIPT_H

//-------------------------------------------------
// QT
//-------------------------------------------------
#include <QThread>
#include <QDir>
#include <QTime>
#include <QDateTime>
#include <QTextStream>

//-------------------------------------------------
// Script Define
//-------------------------------------------------
#include "Def_Script.h"

//-------------------------------------------------
// Mission Class
//-------------------------------------------------
#include "Mission_Class/CDriving/CDriving.h"
#include "Mission_Class/CManipulation/CManipulation.h"

class CScript: public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CScript();
    CScript(CDriving* _p_drivig, CManipulation* _p_manipulation);

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
    CGripper* mpc_gripper;

    //-------------------------------------------------
    // Mission Class
    //-------------------------------------------------
    CDriving* mpc_drivig;
    CManipulation* mpc_manipulation;

private:
    //-------------------------------------------------
    // Interpreter value
    //-------------------------------------------------
    SCENARIO_SCRIPT mstruct_scenario;
    MISSION_SCRIPT* mpary_mission_script;

private:
    //-------------------------------------------------
    // Player value
    //-------------------------------------------------
    SCRIPT_PLAYER_OPTION mstruct_player_option;

private:
    void InitScenarioScript();

public:
    //-------------------------------------------------
    // Interpreter Function
    //-------------------------------------------------
    bool InterpreteScenarioScriptFile(QString _file_name);
    bool InterpreteScenarioScriptLine(QString _line);

    bool InterpreteMissionScriptFile();
    bool InterpreteMissionScriptLine(QString _line, MISSION_SCRIPT* _mission_script, STEP_INFO& _step_info);
    //-------------------------------------------------
    // Function
    //-------------------------------------------------
    bool InterpreteKinovaForceCtrl(QString _line, STEP_INFO& _step_info);
    bool InterpreteKinovaManipulate(QString _line, STEP_INFO& _step_info);
    bool InterpreteGripperForceCtrl(QString _line, STEP_INFO& _step_info);

private:
    //-------------------------------------------------
    // Calculate
    //-------------------------------------------------
    double ReturnKinovaAxisValue(QString _axis, QString _char);
    void SetKinovaAxisValue(KINOVA_DO_MANIPULATE_STRUCT &_manipulat_option);

public:
    //-------------------------------------------------
    // Player Function
    //-------------------------------------------------
    bool SetPlayerOption(SCRIPT_PLAYER_OPTION _player_option);
    bool MissionPlayer();

public:
    void GetScenarioScript(SCENARIO_SCRIPT& _scenario_script);

};

#endif // CSCRIPT_H



