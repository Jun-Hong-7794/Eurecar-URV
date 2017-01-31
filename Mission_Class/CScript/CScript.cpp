#include "CScript.h"

CScript::CScript(){

}

CScript::CScript(CDriving* _p_drivig, CManipulation* _p_manipulation){

    mpc_drivig = _p_drivig;
    mpc_manipulation = _p_manipulation;

    mpary_mission_script = NULL;
}

void CScript::GetStepTitle(int _mission_index, QStringList& _step_title_list){

    _step_title_list.clear();

    for(unsigned int j = 0; j < mpary_mission_script[_mission_index].step_vecor.size(); j++){
        _step_title_list.append(mpary_mission_script[_mission_index].step_vecor.at(j).step_title);
    }
}
//-------------------------------------------------
// Calculate
//-------------------------------------------------
void CScript::SetKinovaAxisValue(KINOVA_DO_MANIPULATE_STRUCT &_manipulat_option){


    _manipulat_option.roll = ReturnKinovaAxisValue("roll", _manipulat_option.str_roll);
    _manipulat_option.pitch = ReturnKinovaAxisValue("pitch", _manipulat_option.str_pitch);
    _manipulat_option.yaw = ReturnKinovaAxisValue("yaw", _manipulat_option.str_yaw);

    _manipulat_option.x = ReturnKinovaAxisValue("x", _manipulat_option.str_x);
    _manipulat_option.y = ReturnKinovaAxisValue("y", _manipulat_option.str_y);
    _manipulat_option.z = ReturnKinovaAxisValue("z", _manipulat_option.str_z);

}

double CScript::ReturnKinovaAxisValue(QString _axis, QString _char){
    CartesianPosition kinova_present_pose;
    mpc_manipulation->KinovaGetPosition(kinova_present_pose);

    if(_axis.contains("roll")){
        if(_char.contains("++")){
            int equal_index = _char.indexOf("++");
            kinova_present_pose.Coordinates.ThetaZ += _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.ThetaZ;
        }
        else if(_char.contains("--")){
            int equal_index = _char.indexOf("--");
            kinova_present_pose.Coordinates.ThetaZ -= _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.ThetaZ;
        }
        else if(_char.contains("==")){
            return kinova_present_pose.Coordinates.ThetaZ;
        }
        else{
            return _char.trimmed().toDouble();
        }
    }
    if(_axis.contains("pitch")){
        if(_char.contains("++")){
            int equal_index = _char.indexOf("++");
            kinova_present_pose.Coordinates.ThetaY += _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.ThetaY;
        }
        else if(_char.contains("--")){
            int equal_index = _char.indexOf("--");
            kinova_present_pose.Coordinates.ThetaY -= _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.ThetaY;
        }
        else if(_char.contains("==")){
            return kinova_present_pose.Coordinates.ThetaY;
        }
        else{
            return _char.trimmed().toDouble();
        }
    }
    if(_axis.contains("yaw")){
        if(_char.contains("++")){
            int equal_index = _char.indexOf("++");
            kinova_present_pose.Coordinates.ThetaX += _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.ThetaX;
        }
        else if(_char.contains("--")){
            int equal_index = _char.indexOf("--");
            kinova_present_pose.Coordinates.ThetaX -= _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.ThetaX;
        }
        else if(_char.contains("==")){
            return kinova_present_pose.Coordinates.ThetaX;
        }
        else{
            return _char.trimmed().toDouble();
        }
    }
    if(_axis.contains("x")){
        if(_char.contains("++")){
            int equal_index = _char.indexOf("++");
            kinova_present_pose.Coordinates.X += _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.X;
        }
        else if(_char.contains("--")){
            int equal_index = _char.indexOf("--");
            kinova_present_pose.Coordinates.X -= _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.X;
        }
        else if(_char.contains("==")){
            return kinova_present_pose.Coordinates.X;
        }
        else{
            return _char.trimmed().toDouble();
        }
    }
    if(_axis.contains("y")){
        if(_char.contains("++")){
            int equal_index = _char.indexOf("++");
            kinova_present_pose.Coordinates.Y += _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.Y;
        }
        else if(_char.contains("--")){
            int equal_index = _char.indexOf("--");
            kinova_present_pose.Coordinates.Y -= _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.Y;
        }
        else if(_char.contains("==")){
            return kinova_present_pose.Coordinates.Y;
        }
        else{
            return _char.trimmed().toDouble();
        }
    }
    if(_axis.contains("z")){
        if(_char.contains("++")){
            int equal_index = _char.indexOf("++");
            kinova_present_pose.Coordinates.Z += _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.Z;
        }
        else if(_char.contains("--")){
            int equal_index = _char.indexOf("--");
            kinova_present_pose.Coordinates.Z -= _char.mid(equal_index + 2).trimmed().toDouble();
            return kinova_present_pose.Coordinates.Z;
        }
        else if(_char.contains("==")){
            return kinova_present_pose.Coordinates.Z;
        }
        else{
            return _char.trimmed().toDouble();
        }
    }


    return 0;
}

//-------------------------------------------------
//
//                 Interprete Fuction
//
//-------------------------------------------------

//-------------------------------------------------
// Interprete Scenario Script
//-------------------------------------------------

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

    InterpreteMissionScriptFile();

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

        QString file_name = _line.mid(colone_index + 1).trimmed();
        QString file_path = mstruct_scenario.mission_file_directory + file_name + ".md";

        mstruct_scenario.mission_file_name.push_back(file_name);
        mstruct_scenario.mission_file_path.push_back(file_path);

        return true;
    }

    return false;
}

void CScript::GetScenarioScript(SCENARIO_SCRIPT& _scenario_script){

    _scenario_script = mstruct_scenario;
}

//-------------------------------------------------
// Interprete Mission Script
//-------------------------------------------------
bool CScript::InterpreteMissionScriptFile(){

    if(mpary_mission_script == NULL)
        mpary_mission_script = new MISSION_SCRIPT[mstruct_scenario.mission_file_name.size()];
    else{
        delete[] mpary_mission_script;
        mpary_mission_script = new MISSION_SCRIPT[mstruct_scenario.mission_file_name.size()];
    }

    for(unsigned int i = 0; i < mstruct_scenario.mission_file_name.size(); i++){

        QString mission_file_path = mstruct_scenario.mission_file_path.at(i);
        QFile inputFile(mission_file_path);

        if (inputFile.open(QIODevice::ReadOnly))
        {
           STEP_INFO step_info;
           step_info.step_title = "Empty";

           QTextStream in(&inputFile);
           while (!in.atEnd())
           {
               InterpreteMissionScriptLine(in.readLine(), &mpary_mission_script[i], step_info);
           }
           inputFile.close();
        }
        else{
            std::cout<< "Mission File Open Error!" << std::endl;
            return false;
        }
    }

    return true;
}

bool CScript::InterpreteMissionScriptLine(QString _line, MISSION_SCRIPT* _mission_script, STEP_INFO& _step_info){

    if(_line.contains("/*"))// '/*' is Comment
        return true;

    if(_line.contains("Title")){

        int colone_index = _line.indexOf(":");

        _mission_script->mission_title = _line.mid(colone_index + 1).trimmed();

        return true;
    }

    if(_line.contains("Step")){

        if(_step_info.step_title != "Empty"){
            _mission_script->step_vecor.push_back(_step_info);
            //Clear _step_info
        }

        _step_info.after__sleep = 0;
        _step_info.before_sleep = 0;

        int colone_index = _line.indexOf(":");

        _step_info.step_title = _line.mid(colone_index + 1).trimmed();

        return true;
    }

    if(_line.contains("A_Sleep")){
        int bracket_small_op_index = _line.indexOf("(");
        int bracket_small_ed_index = _line.indexOf(")");

        _step_info.after__sleep = _line.mid(bracket_small_op_index + 1, bracket_small_ed_index - bracket_small_op_index - 1).toInt();
    }
    if(_line.contains("B_Sleep")){
        int bracket_small_op_index = _line.indexOf("(");
        int bracket_small_ed_index = _line.indexOf(")");

        _step_info.before_sleep = _line.mid(bracket_small_op_index + 1, bracket_small_ed_index - bracket_small_op_index - 1).toInt();
    }

    if(_line.contains("KINOVA_FORCE_CTRL")){

        if(InterpreteKinovaForceCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("KINOVA_MANIPULATE")){

        if(InterpreteKinovaManipulate(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("GRIPPER_FORCE_CTRL")){

        if(InterpreteGripperForceCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("LRF_VEHICLE_HORIZEN_CTRL")){

        if(InterpreteLRFVehicleHorizenCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("LRF_VEHICLE_ANGLE_CTRL")){

        if(InterpreteLRFVehicleAngleCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("MISSION_END")){

        if(_step_info.step_title != "Empty"){
            _mission_script->step_vecor.push_back(_step_info);
        }
        return true;
    }

    return true;
}

bool CScript::InterpreteKinovaForceCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_FORCE_CTRL_STRUCT")){
        if(_line.contains("step_count")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.step_count = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("force_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.forece_threshold = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("move_step_x")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.move_step_x = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("move_step_y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.move_step_y = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("move_step_z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.move_step_z = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
    }
    else if(_line.contains("KINOVA_FORCE_CTRL_FUNCTION")){
        _step_info.function_index = MP_KINOVA_FORCE_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteKinovaManipulate(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_MANIPULATE_STRUCT")){

        if(_line.contains("roll")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.str_roll = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("pitch")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.str_pitch = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("yaw")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.str_yaw = _line.mid(colone_index + 1).trimmed();
            return true;
        }

        else if(_line.contains("force_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.forece_threshold = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("x")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.str_x = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.str_y = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.str_z = _line.mid(colone_index + 1).trimmed();
            return true;
        }

    }
    else if(_line.contains("KINOVA_MANIPULATE_FUNCTION")){
        _step_info.function_index = MP_KINOVA_MANIPULATE;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteGripperForceCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("GRIPPER_FORCE_CTRL_STRUCT")){

        if(_line.contains("bend_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_force_option.bend_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("forece_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_force_option.forece_threshold = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

    }
    else if(_line.contains("GRIPPER_FORCE_CTRL_FUNCTION")){
        _step_info.function_index = MP_GRIPPER_FORCE_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFVehicleAngleCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_VEHICLE_ANGLE_CTRL_STRUCT")){

        if(_line.contains("desired_angle")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_angle_option.desired_angle = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error_boundary")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_angle_option.error_boundary = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_angle_option.s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_angle_option.e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("velocity")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_angle_option.velocity = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

    }
    else if(_line.contains("LRF_VEHICLE_ANGLE_CTRL_FUNCTION")){
        _step_info.function_index = DR_LRF_VEHICLE_ANGLE_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFVehicleHorizenCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_VEHICLE_HORIZEN_CTRL_STRUCT")){

        if(_line.contains("inlier_distance")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_horizen_option.inlier_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("desired_avr_inlier_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_horizen_option.desired_avr_inlier_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error_deg_boundary")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_horizen_option.error_deg_boundary = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("velocity")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.lrf_vehicle_horizen_option.velocity = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

    }
    else if(_line.contains("LRF_VEHICLE_HORIZEN_CTRL_FUNCTION")){
        _step_info.function_index = DR_LRF_VEHICLE_HORIZEN_CONTROL;
    }
    else
        return false;

    return true;
}

//----------------------------------------------------------------
//
//                          Player Function
//
//----------------------------------------------------------------

bool CScript::SetPlayerOption(SCRIPT_PLAYER_OPTION _player_option){

    mstruct_player_option = _player_option;

    if(this->isRunning())//Error: Already running!!
        return false;
    else{
        this->start();
        return true;
    }
}

bool CScript::MissionPlayer(){

    unsigned int step_start_inx = mstruct_player_option.start_step_num;
    unsigned int step_end___inx = mstruct_player_option.end_step_num;

    unsigned int mission_start_inx = mstruct_player_option.start_mission_num < 0 ? 0 : mstruct_player_option.start_mission_num;
    unsigned int mission_end___inx = mstruct_player_option.end_mission_num < 0 ? 0 : mstruct_player_option.end_mission_num;

    if(mission_end___inx > (mstruct_scenario.mission_file_name.size() -1))
            mission_end___inx = (mstruct_scenario.mission_file_name.size() -1);

    QString msessage;

    QString mission_font_option_head = "<font color=red>";
    QString mission_font_option_tail = "</font>";

    QString step_font_option_head = "<font color=blue>";
    QString step_font_option_tail = "</font>";

    for(unsigned int i = mission_start_inx; i <= mission_end___inx; i++){
        if(step_end___inx > (mpary_mission_script[i].step_vecor.size() -1))
                step_end___inx = (mpary_mission_script[i].step_vecor.size() -1);

        msessage = mission_font_option_head;
        msessage += "Start Mission: " + mpary_mission_script[i].mission_title;
        msessage += mission_font_option_tail;

        emit SignalScriptMessage(msessage);

        for(unsigned int j = step_start_inx; j <= step_end___inx; j++){

            msessage = step_font_option_head;
            msessage += "Start Step: " + mpary_mission_script[i].step_vecor.at(j).step_title;
            msessage += step_font_option_tail;

            emit SignalScriptMessage(msessage);

            if(mpary_mission_script[i].step_vecor.at(j).before_sleep != 0)
                msleep(mpary_mission_script[i].step_vecor.at(j).before_sleep);

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_VELODYNE_VEHICLE_CONTROL){
                mpc_drivig->SetDrivingOption(mpary_mission_script[i].step_vecor.at(j).driving_option.driving_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_DRIVE_TO_PANEL);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_LRF_VEHICLE_ANGLE_CONTROL){
                mpc_drivig->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).driving_option.lrf_vehicle_angle_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_LRF_VEHICLE_ANGLE);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_LRF_VEHICLE_HORIZEN_CONTROL){
                mpc_drivig->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).driving_option.lrf_vehicle_horizen_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_LRF_VEHICLE_HORIZEN);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_KINOVA_DEPTH_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_KINOVA);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_FORCE_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_force_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_FORCE_CLRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_GRIPPER_FORCE_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_force_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_FORCE_CLRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_MANIPULATE){
                SetKinovaAxisValue(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_manipulate_option);
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_manipulate_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_MANIPULATE);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).after__sleep != 0)
                msleep(mpary_mission_script[i].step_vecor.at(j).after__sleep);

            msessage ="End Step: " + mpary_mission_script[i].step_vecor.at(j).step_title;
            emit SignalScriptMessage(msessage);
        }

        msessage = mission_font_option_head;
        msessage += "End Mission: " + mpary_mission_script[i].mission_title;
        msessage += mission_font_option_tail;

        emit SignalScriptMessage(msessage);
    }

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CScript::run(){

    MissionPlayer();

}
