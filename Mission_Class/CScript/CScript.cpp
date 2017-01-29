#include "CScript.h"

CScript::CScript(){

}

CScript::CScript(CDriving* _p_drivig, CManipulation* _p_manipulation){

    mpc_drivig = _p_drivig;
    mpc_manipulation = _p_manipulation;
}

//-------------------------------------------------
// Interprete Fuction
//-------------------------------------------------

// Script
void CScript::InitScenarioScript(){

    mstruct_scenario.scenario_title.clear();
    mstruct_scenario.number_of_mission = 0;
    mstruct_scenario.mission_file_name.clear();
    mstruct_scenario.mission_file_directory.clear();
}

bool CScript::InterpreteScenarioScriptFile(QString _file_name){

    InitScenarioScript();

    QFile inputFile(_file_name);

    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
           InterpreteScenarioScriptLine(in.readLine());
       }
       inputFile.close();
    }
    else{
        std::cout<< "Mission File Open Error!" << std::endl;
        return false;
    }

    return true;
}

bool CScript::InterpreteScenarioScriptLine(QString _line){

    if(_line.contains("/*"))// '/*' is Comment
        return true;

    if(_line.contains("Title")){

        int colone_index = _line.indexOf(":");

        mstruct_scenario.scenario_title = _line.mid(colone_index + 1);

        return true;
    }

    if(_line.contains("Mission Directory")){

        int colone_index = _line.indexOf(":");

        mstruct_scenario.mission_file_directory = _line.mid(colone_index + 1).trimmed();

        return true;
    }

    if(_line.contains("Mission File")){

        int colone_index = _line.indexOf(":");

        mstruct_scenario.mission_file_name.push_back(_line.mid(colone_index + 1).trimmed());

        return true;
    }

    return false;
}

void CScript::GetScenarioScript(SCENARIO_SCRIPT& _scenario_script){

    _scenario_script = mstruct_scenario;
}

// Mission
bool CScript::InterpreteMissionScriptFile(QString _file_name){

    QFile inputFile(_file_name);

    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
           InterpreteScenarioScriptLine(in.readLine());
       }
       inputFile.close();
    }
    else{
        std::cout<< "Mission File Open Error!" << std::endl;
        return false;
    }

    return true;
}

bool CScript::InterpreteMissionScriptLine(QString _line){

    if(_line.contains("/*"))// '/*' is Comment
        return true;

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CScript::run(){

}
