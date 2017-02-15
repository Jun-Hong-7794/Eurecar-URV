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

    //-------------------------------------------------
    // Variable Vector
    //-------------------------------------------------
    //Gloabal Variable
    std::vector<SCRIPT_INT> mvec_global_int;
    std::vector<SCRIPT_BOOL> mvec_global_bool;
    std::vector<SCRIPT_DOUBLE> mvec_global_double;

private:
    void InitScenarioScript();

    QMutex mtx_mission_pause;
    QMutex mtx_mission_terminate;

    bool fl_mission_pause;
    bool fl_mission_terminate;
public:
    //-------------------------------------------------
    // Mission Pause/Terminate Flag Setting  Function
    //-------------------------------------------------
    void SetMissionPause(bool _pause);
    void SetMissionTerminate(bool _terminate);

    bool IsMissionPaused();
    bool IsMissionTerminated();

    //-------------------------------------------------
    // Interpreter Function
    //-------------------------------------------------
    bool InterpreteScenarioScriptFile(QString _file_name);
    bool InterpreteScenarioScriptLine(QString _line);

private:
    //-------------------------------------------------
    // Interprete Global Value
    //-------------------------------------------------
    bool InterpreteGlobalValue(QString _line);

    //-------------------------------------------------
    // Interprete Local Value
    //-------------------------------------------------
    bool InterpreteLocalValue(QString _line, MISSION_SCRIPT* _mission_script);

    //-------------------------------------------------
    bool InterpreteMissionScriptFile();
    bool InterpreteMissionScriptLine(QString _line, MISSION_SCRIPT* _mission_script, STEP_INFO& _step_info);
    //-------------------------------------------------
    // Function
    //-------------------------------------------------
    bool InterpreteVehicleDriveToPanel(QString _line, STEP_INFO& _step_info);
    bool InterpreteVehicleParking(QString _line, STEP_INFO& _step_info);

    bool InterpreteKinovaForceCtrl(QString _line, STEP_INFO& _step_info);
    bool InterpreteKinovaForceCheck(QString _line, STEP_INFO& _step_info);
    bool InterpreteKinovaManipulate(QString _line, STEP_INFO& _step_info);
    bool InterpreteKinovaRotateValveCtrl(QString _line, STEP_INFO& _step_info);

    bool InterpreteGripperForceCtrl(QString _line, STEP_INFO& _step_info);
    bool InterpreteGripperMagnetCtrl(QString _line, STEP_INFO& _step_info);
    bool InterpreteGripperValveSizeRecog(QString _line, STEP_INFO& _step_info);

    bool InterpreteLRFKinovaVerticalCtrl(QString _line, STEP_INFO& _step_info);
    bool InterpreteLRFKinovaHorizenCtrl(QString _line, STEP_INFO& _step_info);

    bool InterpreteLRFVehicleAngleCtrl(QString _line, STEP_INFO& _step_info);
    bool InterpreteLRFVehicleHorizenCtrl(QString _line, STEP_INFO& _step_info);

    bool InterpreteWrenchRecognition(QString _line, STEP_INFO& _step_info);

    //-------------------------------------------------
    // Grammar
    //-------------------------------------------------
    bool SetIntVariable(QString _str_variable, MISSION_SCRIPT& _mission_script, int _value);
    bool SetBoolVariable(QString _str_variable, MISSION_SCRIPT& _mission_script, bool _result);
    bool SetDoubleVariable(QString _str_variable, MISSION_SCRIPT& _mission_script, double _value);

    bool InterpreteConditionallyIterate(QString _line, STEP_INFO& _step_info);

    int InterpreteIntVariable(QString _line, MISSION_SCRIPT _mission_script/*For Local Variable*/);
    bool InterpreteBoolVariable(QString _line, MISSION_SCRIPT _mission_script/*For Local Variable*/);
    double InterpreteDoubleVariable(QString _line, MISSION_SCRIPT _mission_script/*For Local Variable*/);

private:
    //-------------------------------------------------
    // Calculate
    //-------------------------------------------------
    double ReturnKinovaAxisValue(QString _axis, QString _char);
    void SetKinovaAxisValue(KINOVA_DO_MANIPULATE_STRUCT &_manipulat_option);

public:
    void GetStepTitle(int _mission_index, QStringList& _step_title_list);

public:
    //-------------------------------------------------
    // Player Function
    //-------------------------------------------------
    bool SetPlayerOption(SCRIPT_PLAYER_OPTION _player_option);
    bool MissionPlayer();

public:
    void GetScenarioScript(SCENARIO_SCRIPT& _scenario_script);

signals:
    void SignalScriptMessage(QString _msessage);

};

#endif // CSCRIPT_H



