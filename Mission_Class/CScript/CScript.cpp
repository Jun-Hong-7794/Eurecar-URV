#include "CScript.h"

CScript::CScript(){

}

CScript::CScript(CDriving* _p_drivig, CManipulation* _p_manipulation){

    mpc_drivig = _p_drivig;
    mpc_manipulation = _p_manipulation;

    mpary_mission_script = NULL;

    fl_mission_pause = false;
    fl_mission_terminate = false;
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

void CScript::SetKinovaAxisValue(KINOVA_CURRENT_POSITION &_manipulat_option, MISSION_SCRIPT& _mission_script){

    CartesianPosition kinova_present_pose;
    mpc_manipulation->KinovaGetPosition(kinova_present_pose);

    SetDoubleVariable(_manipulat_option.str_roll, _mission_script, kinova_present_pose.Coordinates.ThetaZ);
    SetDoubleVariable(_manipulat_option.str_pitch, _mission_script, kinova_present_pose.Coordinates.ThetaY);
    SetDoubleVariable(_manipulat_option.str_yaw, _mission_script, kinova_present_pose.Coordinates.ThetaX);

    SetDoubleVariable(_manipulat_option.str_x, _mission_script, kinova_present_pose.Coordinates.X);
    SetDoubleVariable(_manipulat_option.str_y, _mission_script, kinova_present_pose.Coordinates.Y);
    SetDoubleVariable(_manipulat_option.str_z, _mission_script, kinova_present_pose.Coordinates.Z);

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

void CScript::SetMissionPause(bool _pause){

    mtx_mission_pause.lock();
    {
        fl_mission_pause = _pause;
    }
    mtx_mission_pause.unlock();
}

void CScript::SetMissionTerminate(bool _terminate){

    mtx_mission_terminate.lock();
    {
        fl_mission_terminate = _terminate;
    }
    mtx_mission_terminate.unlock();
}

bool CScript::IsMissionPaused(){

    bool pause;

    mtx_mission_pause.lock();
    {
        pause = fl_mission_pause;
    }
    mtx_mission_pause.unlock();

    return pause;
}

bool CScript::IsMissionTerminated(){

    bool terminate;

    mtx_mission_terminate.lock();
    {
        terminate = fl_mission_terminate;
    }
    mtx_mission_terminate.unlock();

    return terminate;
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

    mvec_global_bool.clear();
    mvec_global_int.clear();
    mvec_global_double.clear();
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
        _step_info.fl_condition_if_flag = false;
        _step_info.fl_condition_else_flag = false;

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

    if(_line.contains("IF(")){
        int bracket_small_op_index = _line.indexOf("(");
        int bracket_small_ed_index = _line.indexOf(")");

        if((bracket_small_op_index == -1) || (bracket_small_ed_index == -1))
            return false;

        _step_info.str_if = _line.mid(bracket_small_op_index + 1, bracket_small_ed_index - bracket_small_op_index - 1);
        _step_info.fl_condition_if_flag = true;
    }

    if(_line.contains("ELSE(")){
        int bracket_small_op_index = _line.indexOf("(");
        int bracket_small_ed_index = _line.indexOf(")");

        if((bracket_small_op_index == -1) || (bracket_small_ed_index == -1))
            return false;

        _step_info.str_else = _line.mid(bracket_small_op_index + 1, bracket_small_ed_index - bracket_small_op_index - 1);
        _step_info.fl_condition_else_flag = true;
    }

    if(_line.contains("global_")){

        if(InterpreteGlobalValue(_line))
            return true;
    }

    if(_line.contains("local_")){

        if(InterpreteLocalValue(_line, _mission_script))
            return true;
    }


    if(_line.contains("DRIVE_BY_WRENCH_RECOG")){
        if(InterpreteDriveByWrenchRecog(_line, _step_info))
            return true;
        else
            return false;
    }
    if(_line.contains("CONDITIONALLY_ITERATION")){
        if(InterpreteConditionallyIterate(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("VEHICLE_DRIVE_TO_PANEL")){

        if(InterpreteVehicleDriveToPanel(_line, _step_info))
            return true;
        else
            return false;
    }
    if(_line.contains("VEHICLE_LOCALIZATION_ON_PANEL")){

        if(InterpreteVehicleLocalizationOnPanel(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("VEHICLE_PARKING")){

        if(InterpreteVehicleParking(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("KINOVA_FORCE_CTRL")){

        if(InterpreteKinovaForceCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("KINOVA_FORCE_CHECK")){

        if(InterpreteKinovaForceCheck(_line, _step_info))
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
    if(_line.contains("KINOVA_ALIGN_TO_PANEL")){

        if(InterpreteKinovaAlignToPanel(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("KINOVA_FIT_TO_VALVE_POSE")){

        if(InterpreteKinovaFitToValvePose(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("KINOVA_ROTATE_VALVE")){

        if(InterpreteKinovaRotateValveCtrl(_line, _step_info))
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
    if(_line.contains("GRIPPER_GO_TO_REL_POSE")){

        if(InterpreteGripperGotoRelPose(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("GRIPPER_MAGNET_CTRL")){

        if(InterpreteGripperMagnetCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("VALVE_SIZE_RECOG")){

        if(InterpreteGripperValveSizeRecog(_line, _step_info))
            return true;
        else
            return false;
    }


    // New
    if(_line.contains("LRF_K_VERTICAL_CTRL")){

        if(InterpreteLRFKVerticalCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("LRF_K_HORIZEN_CTRL")){

        if(InterpreteLRFKHorizenCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("LRF_K_ANGLE_CTRL")){

        if(InterpreteLRFKAngleCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("LRF_V_ANGLE_CTRL")){

        if(InterpreteLRFVAngleCtrl(_line, _step_info))
            return true;
        else
            return false;
    }
    if(_line.contains("KINOVA_BY_WRENCH_RECOG")){

        if(InterpreteKinovaByWrenchRecog(_line, _step_info))
            return true;
        else
            return false;
    }



    if(_line.contains("LRF_V_HORIZEN_CTRL")){

        if(InterpreteLRFVHorizenCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("KINOVA_CURRENT_POSITION")){

        if(InterpreteKinovaCurrentPosition(_line, _step_info))
            return true;
        else
            return false;
    }

    // Old
    if(_line.contains("LRF_KINOVA_VERTICAL_CTRL")){

        if(InterpreteLRFKinovaVerticalCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("LRF_KINOVA_HORIZEN_CTRL")){

        if(InterpreteLRFKinovaHorizenCtrl(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("LRF_KINOVA_WRENCH_LOCATION_CTRL")){

        if(InterpreteLRFKinovaWrenchLocationCtrl(_line, _step_info))
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

    if(_line.contains("ROTATOR")){

        if(InterpreteRotator(_line, _step_info))
            return true;
//        else
//            return false;
    }

    if(_line.contains("CHECK_CURRENT_V_DISTANCE")){

        if(InterpreteCheckCurrentVDst(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("PARKING_RETRY")){

        if(InterpreteParkingRetry(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("WRENCH_RECOGNITION")){

        if(InterpreteWrenchRecognition(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("KINOVA_LRF_VALVE_SEARCHING")){

        if(InterpreteKinovaLRFValveSearching(_line, _step_info))
            return true;
        else
            return false;
    }

    if(_line.contains("FIND_VALVE_LOCATION")){

        if(InterpreteGripperFindValveLocation(_line, _step_info))
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

bool CScript::InterpreteGlobalValue(QString _line){

    QString global_int = "global_int";
    QString global_bool = "global_bool";
    QString global_double = "global_double";

    int int_index = _line.indexOf(global_int);
    int bool_index = _line.indexOf(global_bool);
    int double_index = _line.indexOf(global_double);

    if(double_index >= 0){
        QString str_value;
        QString str_variable_name;
        SCRIPT_DOUBLE double_value;

        str_variable_name = _line.mid(double_index + global_double.size()).trimmed();

        int equal_index = str_variable_name.indexOf("=");

        if(equal_index >= 0){

            str_value = str_variable_name.mid(equal_index + 1).trimmed();
            str_variable_name = str_variable_name.mid(0, equal_index - 1).trimmed();

            double_value.double_value = str_value.toDouble();
            double_value.variable_name = str_variable_name;
        }
        else{
            double_value.double_value = 0;
            double_value.variable_name = str_variable_name;
        }

        mvec_global_double.push_back(double_value);
    }
    if(int_index >= 0){
        QString str_value;
        QString str_variable_name;
        SCRIPT_INT int_value;

        str_variable_name = _line.mid(int_index + global_int.size()).trimmed();

        int equal_index = str_variable_name.indexOf("=");

        if(equal_index >= 0){

            str_value = str_variable_name.mid(equal_index + 1).trimmed();
            str_variable_name = str_variable_name.mid(0, equal_index - 1).trimmed();

            int_value.int_value = str_value.toInt();
            int_value.variable_name = str_variable_name;
        }
        else{
            int_value.int_value = 0;
            int_value.variable_name = str_variable_name;
        }
        mvec_global_int.push_back(int_value);
    }
    if(bool_index >= 0){

        QString str_value;
        QString str_variable_name;
        SCRIPT_BOOL bool_value;

        str_variable_name = _line.mid(bool_index + global_bool.size()).trimmed();

        int equal_index = str_variable_name.indexOf("=");

        if(equal_index >= 0){

            str_value = str_variable_name.mid(equal_index + 1).trimmed();
            str_variable_name = str_variable_name.mid(0, equal_index - 1).trimmed();

            if(str_value.contains("true")){
                bool_value.bool_value = true;
            }
            else{
                bool_value.bool_value = false;
            }
            bool_value.variable_name = str_variable_name;
        }
        else{
            bool_value.bool_value = 0;
            bool_value.variable_name = str_variable_name;
        }
        mvec_global_bool.push_back(bool_value);
    }

    else
        return false;

    return true;
}

bool CScript::InterpreteLocalValue(QString _line, MISSION_SCRIPT* _mission_script){

    QString local_int = "local_int";
    QString local_bool = "local_bool";
    QString local_double = "local_double";

    int int_index = _line.indexOf(local_int);
    int bool_index = _line.indexOf(local_bool);
    int double_index = _line.indexOf(local_double);

    if(double_index >= 0){
        QString str_value;
        QString str_variable_name;
        SCRIPT_DOUBLE double_value;

        str_variable_name = _line.mid(double_index + local_double.size()).trimmed();

        int equal_index = str_variable_name.indexOf("=");

        if(equal_index >= 0){

            str_value = str_variable_name.mid(equal_index + 1).trimmed();
            str_variable_name = str_variable_name.mid(0, equal_index - 1).trimmed();

            double_value.double_value = str_value.toDouble();
            double_value.variable_name = str_variable_name;
        }
        else{
            double_value.double_value = 0;
            double_value.variable_name = str_variable_name;
        }
        _mission_script->vec_lc_double.push_back(double_value);
    }
    else if(int_index >= 0){
        QString str_value;
        QString str_variable_name;
        SCRIPT_INT int_value;

        str_variable_name = _line.mid(int_index + local_int.size()).trimmed();

        int equal_index = str_variable_name.indexOf("=");

        if(equal_index >= 0){

            str_value = str_variable_name.mid(equal_index + 1).trimmed();
            str_variable_name = str_variable_name.mid(0, equal_index - 1).trimmed();

            int_value.int_value = str_value.toInt();
            int_value.variable_name = str_variable_name;
        }
        else{
            int_value.int_value = 0;
            int_value.variable_name = str_variable_name;
        }
        _mission_script->vec_lc_int.push_back(int_value);
    }
    else if(bool_index >= 0){
        QString str_value;
        QString str_variable_name;
        SCRIPT_BOOL bool_value;

        str_variable_name = _line.mid(bool_index + local_bool.size()).trimmed();

        int equal_index = str_variable_name.indexOf("=");

        if(equal_index >= 0){

            str_value = str_variable_name.mid(equal_index + 1).trimmed();
            str_variable_name = str_variable_name.mid(0, equal_index - 1).trimmed();

            if(str_value.contains("true")){
                bool_value.bool_value = true;
            }
            else{
                bool_value.bool_value = false;
            }
            bool_value.variable_name = str_variable_name;
        }
        else{
            bool_value.bool_value = 0;
            bool_value.variable_name = str_variable_name;
        }
        _mission_script->vec_lc_bool.push_back(bool_value);
    }
    else
        return false;

    return true;
}
bool CScript::InterpreteDriveByWrenchRecog(QString _line, STEP_INFO& _step_info){

    if(_line.contains("DRIVE_BY_WRENCH_RECOG_STRUCT")){
        if(_line.contains("desired_wrench_index")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.wrench_recog_option.str_desired_wrench_index = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("lrf_v_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.wrench_recog_option.str_lrf_v_dst = _line.mid(colone_index + 1).trimmed();
            return true;
        }
    }

    else if(_line.contains("DRIVE_BY_WRENCH_RECOG_FUNCTION")){
        _step_info.function_index = DR_WRENCH_RECOG ;
    }
    return true;

}

bool CScript::InterpreteVehicleDriveToPanel(QString _line, STEP_INFO& _step_info){

    if(_line.contains("VEHICLE_DRIVE_TO_PANEL_STRUCT")){
        if(_line.contains("velocity")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.velocity = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("drive_left")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.drive_left = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("drive_differ_left")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.drive_differ_left = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("drive_right")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.drive_right = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("drive_differ_right")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.drive_differ_right = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("parking_left")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_left = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("parking_right")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_right = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("parking_differ_right")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_differ_right = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("parking_differ_left")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_differ_left = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("parking_forward")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_forward = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("parking_backward")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_backward = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("panel_center_check_point")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.panel_center_check_point = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("parking_dist")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_dist = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("parking_thres")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.parking_thres = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("side_center_margin")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.side_center_margin = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("max_vel_script")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.max_vel_script = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("min_vel_script")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.driving_option.min_vel_script = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

    }

    else if(_line.contains("VEHICLE_DRIVE_TO_PANEL_FUNCTION")){
        _step_info.function_index = DR_VEHICLE_DRIVE_TO_PANEL;
    }
    return true;
}
bool CScript::InterpreteVehicleLocalizationOnPanel(QString _line, STEP_INFO& _step_info){

    if(_line.contains("VEHICLE_LOCALIZATION_ON_PANEL_STRUCT")){
        if(_line.contains("desired_h_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.vehicle_localization_option.desired_h_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
    }

    else if(_line.contains("VEHICLE_LOCALIZATION_ON_PANEL_FUNCTION")){
        _step_info.function_index = DR_LOCALIZATION ;
    }
    return true;
}

bool CScript::InterpreteVehicleParking(QString _line, STEP_INFO& _step_info){

    if(_line.contains("VEHICLE_PARKING_STRUCT")){
        if(_line.contains("velocity")){
//            int colone_index = _line.indexOf("=");
//            _step_info.driving_option.parking_option = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
    }

    else if(_line.contains("VEHICLE_PARKING_FUNCTION")){
        _step_info.function_index = DR_VEHICLE_PARKING;
    }
    return true;
}

bool CScript::InterpreteKinovaForceCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_FORCE_CTRL_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.threshold_mode = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        else if(_line.contains("step_count")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.step_count = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        else if(_line.contains("force_range_s")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.force_range_s = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("force_range_e")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.force_range_e = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        else if(_line.contains("force_threshold_x")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.force_threshold_x = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("force_threshold_y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.force_threshold_y = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("force_threshold_z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.force_threshold_z = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        else if(_line.contains("position_limit_x")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.position_limit_x = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("position_limit_y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.position_limit_y = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        else if(_line.contains("position_limit_z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.position_limit_z = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        else if(_line.contains("force_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_option.force_threshold = _line.mid(colone_index + 1).trimmed().toDouble();
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
        int equal_index = _line.indexOf("=");

        if(equal_index >= 0){
            _step_info.manipulation_option.kinova_force_option.str_result_variable =
                    _line.mid(0, equal_index - 1).trimmed();
        }
        _step_info.function_index = MP_KINOVA_FORCE_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteKinovaForceCheck(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_FORCE_CHECK_STRUCT")){

        if(_line.contains("force_threshold_x")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_check_option.force_threshold_x = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("force_threshold_y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_check_option.force_threshold_y = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("force_threshold_z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_check_option.force_threshold_z = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("check_count")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_check_option.check_count = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("check_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_force_check_option.check_threshold = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
    }
    else if(_line.contains("KINOVA_FORCE_CHECK_FUNCTION")){
        int equal_index = _line.indexOf("=");

        if(equal_index >= 0){
            _step_info.manipulation_option.kinova_force_check_option.str_result_variable =
                    _line.mid(0, equal_index - 1).trimmed();
        }
        _step_info.function_index = MP_KINOVA_FORCE_CHECK;
    }
    else
        return false;
    return true;
}

bool CScript::InterpreteKinovaManipulate(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_MANIPULATE_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_manipulate_option.mode = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

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
            _step_info.manipulation_option.kinova_manipulate_option.force_threshold = _line.mid(colone_index + 1).trimmed().toDouble();
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

bool CScript::InterpreteKinovaAlignToPanel(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_ALIGN_TO_PANEL")){
        if(_line.contains("do_init_motion")){
            int colone_index = _line.indexOf("=");
            QString str_option;

            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.fl_do_init_motion = true;
            else
              _step_info.fl_do_init_motion = false;
        }
    }
    if(_line.contains("KINOVA_ALIGN_TO_PANEL_FUNCTION")){
        _step_info.function_index = MP_KINOVA_ALIGN_TO_PANEL;
    }

    return true;
}

bool CScript::InterpreteKinovaFitToValvePose(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_FIT_TO_VALVE_POSE_STRUCT")){

        if(_line.contains("valve_size")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_fit_to_valve_pose.str_valve_size = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("valve_rotation_angle")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_fit_to_valve_pose.str_rotation_angle = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("move_step")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_fit_to_valve_pose.move_step = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("angle_step")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_fit_to_valve_pose.angle_step = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
    }

    if(_line.contains("KINOVA_FIT_TO_VALVE_POSE_FUNCTION")){
        _step_info.function_index = MP_KINOVA_FIT_TO_VALVE_POSE;
    }

    return true;
}

bool CScript::InterpreteGripperForceCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("GRIPPER_FORCE_CTRL_STRUCT")){

        if(_line.contains("bend_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_force_option.bend_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("pose_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_force_option.pose_1 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("pose_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_force_option.pose_2 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("force_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_force_option.force_threshold = _line.mid(colone_index + 1).trimmed().toDouble();
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

bool CScript::InterpreteGripperGotoRelPose(QString _line, STEP_INFO& _step_info){

    if(_line.contains("GRIPPER_GO_TO_REL_POSE_STRUCT")){

        if(_line.contains("pose_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.grippper_go_to_rel_pose_option.pose_1 = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("pose_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.grippper_go_to_rel_pose_option.pose_2 = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
    }
    else if(_line.contains("GRIPPER_GO_TO_REL_POSE_FUNCTION")){
        _step_info.function_index = MP_GRIPPER_GO_TO_REL_POSE;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteGripperMagnetCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("GRIPPER_MAGNET_CTRL_STRUCT")){

        if(_line.contains("fl_magnet")){
            int colone_index = _line.indexOf("=");
            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.gripper_magnet_option.fl_magnet = true;
            else
                _step_info.manipulation_option.gripper_magnet_option.fl_magnet = false;

            return true;
        }
    }
    else if(_line.contains("GRIPPER_MAGNET_CTRL_FUNCTION")){
        _step_info.function_index = MP_GRIPPER_MAGNET_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteWrenchRecognition(QString _line, STEP_INFO& _step_info){

    if(_line.contains("WRENCH_RECOGNITION_STRUCT")){

        if(_line.contains("valve_size")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.wrench_recognition_option.str_valve_size = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("num_of_wrench")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.wrench_recognition_option.str_num_of_wrench = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("loop_count")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.wrench_recognition_option.loop_count = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
    }
    else if(_line.contains("WRENCH_RECOGNITION_FUNCTION")){
        int equal_index = _line.indexOf("=");

        if(equal_index >= 0){
            _step_info.manipulation_option.wrench_recognition_option.str_result_variable =
                    _line.mid(0, equal_index - 1).trimmed();
        }
        _step_info.function_index = MP_WRENCH_RECOGNITION;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteKinovaLRFValveSearching(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_LRF_VALVE_SEARCHING_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.mode =
                    _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.s_deg =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.e_deg =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("maximum_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.maximum_lrf_dst =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("searching_lengh")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.searching_length =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("num_point_thresh")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.num_point_thresh =
                    _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("height_thresh")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.height_thresh =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("offset")){/*cm*/
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.offset =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("loop_sleep")){/*cm*/
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_lrf_valve_searching_option.loop_sleep =
                    _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
    }

    else if(_line.contains("KINOVA_LRF_VALVE_SEARCHING_FUNCTION")){
        _step_info.function_index = MP_KINOVA_LRF_VALVE_SEARCHING;
    }

    else
        return false;

    return true;
}

bool CScript::InterpreteGripperFindValveLocation(QString _line, STEP_INFO& _step_info){

    if(_line.contains("FIND_VALVE_LOCATION_STRUCT")){

        if(_line.contains("grap_error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.grap_error =
                    _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("grasp_pose_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.grasp_pose_1 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("grasp_pose_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.grasp_pose_2 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("release_pose_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.release_pose_1 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("release_pose_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.release_pose_2 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("force_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.force_threshold =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("trial")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.trial =
                    _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("valve_size")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.str_valve_size  =
                    _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("valve_rotation")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_location_option.str_valve_rotation  =
                    _line.mid(colone_index + 1).trimmed();
            return true;
        }
    }
   else if(_line.contains("FIND_VALVE_LOCATION_FUNCTION")){
        _step_info.function_index = MP_GRIPPER_FIND_VALVE_LOCATION;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteGripperValveSizeRecog(QString _line, STEP_INFO& _step_info){

    if(_line.contains("VALVE_SIZE_RECOG_STRUCT")){

        if(_line.contains("grasp_pose_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.grasp_pose_1 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("grasp_pose_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.grasp_pose_2 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("release_pose_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.release_pose_1 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("release_pose_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.release_pose_2 =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("force_threshold")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.force_threshold =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("trial")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.trial =
                    _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("rotation_angle")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.rotation_angle =
                    _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("retry_num")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.retry_num =
                    _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
    }
    else if(_line.contains("VALVE_SIZE_RECOG_GET_SIZE")){

        int equal_index = _line.indexOf("=");

        if(equal_index >= 0){
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.str_size_result_variable =
                    _line.mid(0, equal_index - 1).trimmed();
        }
    }
    else if(_line.contains("VALVE_SIZE_RECOG_GET_ANGLE")){

        int equal_index = _line.indexOf("=");

        if(equal_index >= 0){
            _step_info.manipulation_option.gripper_kinova_valve_recog_option.str_rotation_result_variable =
                    _line.mid(0, equal_index - 1).trimmed();
        }
    }
    else if(_line.contains("VALVE_SIZE_RECOG_FUNCTION")){
        _step_info.function_index = MP_GRIPPER_VALVE_SIZE_RECOG;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteRotator(QString _line, STEP_INFO& _step_info){

    if(_line.contains("ROTATOR_STRUCT")){

        if(_line.contains("desired_position")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.rotator_option.desired_position = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("fl_rotator_torque")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.rotator_option.fl_torque_on_off = true;
            else
                _step_info.manipulation_option.rotator_option.fl_torque_on_off = false;

            return true;
        }

    }
    else if(_line.contains("ROTATOR_FUNCTION")){
        _step_info.function_index = MP_ROTATOR;
    }

    return true;
}

bool CScript::InterpreteParkingRetry(QString _line, STEP_INFO& _step_info){

    if(_line.contains("PARKING_RETRY_STRUCT")){

        if(_line.contains("bias")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.parking_retry_option.str_bias = _line.mid(colone_index + 1).trimmed();
            return true;
        }

        if(_line.contains("max_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.parking_retry_option.max_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("min_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.parking_retry_option.min_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("desired_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.parking_retry_option.desired_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error_bound")){
            int colone_index = _line.indexOf("=");
            _step_info.driving_option.parking_retry_option.error_bound = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
    }
    else if(_line.contains("PARKING_RETRY_FUNCTION")){
        _step_info.function_index = DR_PARKING_RETRY;
    }
    return true;
}

bool CScript::InterpreteCheckCurrentVDst(QString _line, STEP_INFO& _step_info){

    if(_line.contains("CHECK_CURRENT_V_DISTANCE_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");

            if(_line.mid(colone_index + 1).trimmed().toInt() == 1){
                _step_info.manipulation_option.check_v_distance_option.mode = L_M_ROUGH;
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 2){
                _step_info.manipulation_option.check_v_distance_option.mode = (L_M_PRECISE | L_M_DIR_LEFT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 3){
                _step_info.manipulation_option.check_v_distance_option.mode = (L_M_PRECISE | L_M_DIR_RIGHT | L_M_VALUE_RANSAC);
            }

            return true;
        }

        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.check_v_distance_option.s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.check_v_distance_option.e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("maximum_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.check_v_distance_option.maximum_lrf_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("desired_v_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.check_v_distance_option.desired_v_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("error_bound")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.check_v_distance_option.error_bound = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

    }
    else if(_line.contains("GET_CHECK_CURRENT_V_DISTANCE_BIAS")){
        int equal_index = _line.indexOf("=");

        if(equal_index >= 0){
            _step_info.manipulation_option.check_v_distance_option.str_bias=
                    _line.mid(0, equal_index - 1).trimmed();
        };
    }
    else if(_line.contains("CHECK_CURRENT_V_DISTANCE_FUNCTION")){
        _step_info.function_index = MP_KINOVA_LRF_CHECK_V_DST;

        int equal_index = _line.indexOf("=");

        if(equal_index >= 0){
            _step_info.manipulation_option.check_v_distance_option.str_result=
                    _line.mid(0, equal_index - 1).trimmed();
        };
    }

    return true;
}

bool CScript::InterpreteLRFVehicleAngleCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_VEHICLE_ANGLE_CTRL_STRUCT")){

        if(_line.contains("desired_angle")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_angle_option.desired_angle = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error_boundary")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_angle_option.error_boundary = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_angle_option.s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_angle_option.e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("velocity")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_angle_option.velocity = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("sensor_option")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_k_vehicle_angle_option.sensor_option = true;
            else
                _step_info.manipulation_option.lrf_k_vehicle_angle_option.sensor_option = false;

            return true;
        }

    }
    else if(_line.contains("LRF_VEHICLE_ANGLE_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_K_VEHICLE_ANGLE_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFVehicleHorizenCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_VEHICLE_HORIZEN_CTRL_STRUCT")){

        if(_line.contains("inlier_distance")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_horizen_option.inlier_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("desired_avr_inlier_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_horizen_option.desired_avr_inlier_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error_deg_boundary")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_horizen_option.error_deg_boundary = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("velocity")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_horizen_option.velocity = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_horizen_option.s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_vehicle_horizen_option.e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("sensor_option")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_k_vehicle_horizen_option.sensor_option = true;
            else
                _step_info.manipulation_option.lrf_k_vehicle_horizen_option.sensor_option = false;

            return true;
        }

    }
    else if(_line.contains("LRF_VEHICLE_HORIZEN_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_K_VEHICLE_HORIZEN_CONTROL;
    }
    else
        return false;

    return true;
}

//--------------------------------
//New LRF Kinova Vertical Control
//--------------------------------

bool CScript::InterpreteLRFKVerticalCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_K_VERTICAL_CTRL_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");

            if(_line.mid(colone_index + 1).trimmed().toInt() == 1){
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.lrf_info_struct.mode = L_M_ROUGH;
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 2){
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_LEFT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 3){
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_RIGHT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 4){
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.lrf_info_struct.mode = 4;
            }
            return true;
        }

        if(_line.contains("desired_v_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.desired_v_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.lrf_info_struct
                    .s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.lrf_info_struct
                    .e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.lrf_info_struct
                    .inlier_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.loop_sleep = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("only_sensing_moving")){

            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.fl_only_sensing_moving  = true;
            else
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.fl_only_sensing_moving  = false;

            return true;
        }

        if(_line.contains("force_option")){

            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.fl_force_option  = true;
            else
                _step_info.manipulation_option.lrf_k_v_ctrl_struct.fl_force_option  = false;

            return true;
        }
        if(_line.contains("force_x")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.force_x = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("force_y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.force_y = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("force_z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_v_ctrl_struct.force_z = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }


    }
    else if(_line.contains("LRF_K_VERTICAL_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_K_VERTIVAL_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFKHorizenCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_K_HORIZEN_CTRL_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");
            if(_line.mid(colone_index + 1).trimmed().toInt() == 1){
                _step_info.manipulation_option.lrf_k_h_ctrl_struct.lrf_info_struct.mode = L_M_ROUGH;
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 2){
                _step_info.manipulation_option.lrf_k_h_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_LEFT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 3){
                _step_info.manipulation_option.lrf_k_h_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_RIGHT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 4){
                _step_info.manipulation_option.lrf_k_h_ctrl_struct.lrf_info_struct.mode = 4;
            }
            return true;
        }

        if(_line.contains("wrench_hanger_index")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.wrench_hanger_index_str = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("wrench_location_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.wrench_location_1 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.wrench_location_2 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_3")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.wrench_location_3 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_4")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.wrench_location_4 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_5")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.wrench_location_5 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_6")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.wrench_location_6 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("desired_h_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.desired_h_location = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.lrf_info_struct
                    .s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.lrf_info_struct
                    .e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.lrf_info_struct
                    .inlier_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.loop_sleep = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

        if(_line.contains("kinova_current_y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_h_ctrl_struct.str_kinova_current_y = _line.mid(colone_index + 1).trimmed();
            return true;
        }

    }
    else if(_line.contains("LRF_K_HORIZEN_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_K_HORIZEN_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFKAngleCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_K_ANGLE_CTRL_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");
            if(_line.mid(colone_index + 1).trimmed().toInt() == 1){
                _step_info.manipulation_option.lrf_k_a_ctrl_struct.lrf_info_struct.mode = L_M_ROUGH;
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 2){
                _step_info.manipulation_option.lrf_k_a_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_LEFT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 3){
                _step_info.manipulation_option.lrf_k_a_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_RIGHT | L_M_VALUE_RANSAC);
            }
            return true;
        }

        if(_line.contains("desired_angle")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_a_ctrl_struct.desired_angle = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_a_ctrl_struct.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_a_ctrl_struct.lrf_info_struct
                    .s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_a_ctrl_struct.lrf_info_struct
                    .e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_a_ctrl_struct.lrf_info_struct
                    .inlier_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_a_ctrl_struct.loop_sleep = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("unit_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_k_a_ctrl_struct.unit_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

    }
    else if(_line.contains("LRF_K_ANGLE_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_K_ANGLE_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFVAngleCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_V_ANGLE_CTRL_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");
            if(_line.mid(colone_index + 1).trimmed().toInt() == 1){
                _step_info.manipulation_option.lrf_v_a_ctrl_struct.lrf_info_struct.mode = L_M_ROUGH;
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 2){
                _step_info.manipulation_option.lrf_v_a_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_LEFT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 3){
                _step_info.manipulation_option.lrf_v_a_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_RIGHT | L_M_VALUE_RANSAC);
            }
            return true;
        }

        if(_line.contains("desired_angle")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_a_ctrl_struct.desired_angle = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_a_ctrl_struct.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_a_ctrl_struct.lrf_info_struct
                    .s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_a_ctrl_struct.lrf_info_struct
                    .e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_a_ctrl_struct.lrf_info_struct
                    .inlier_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_a_ctrl_struct.loop_sleep = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("velocity")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_a_ctrl_struct.velocity = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("goto_home_pose")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_v_a_ctrl_struct.fl_goto_home_pose = true;
            else
                _step_info.manipulation_option.lrf_v_a_ctrl_struct.fl_goto_home_pose = false;

            return true;

        }

    }
    else if(_line.contains("LRF_V_ANGLE_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_V_ANGLE_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteKinovaByWrenchRecog(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_BY_WRENCH_RECOG_STRUCT")){

        if(_line.contains("desired_wrench_index")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.wrench_searching_option.str_desired_wrench_index = _line.mid(colone_index + 1).trimmed();
            return true;
        }
    }
    else if(_line.contains("KINOVA_BY_WRENCH_RECOG_FUNCTION")){
        _step_info.function_index = MP_KINOVA_WRENCH_SEARCHING;
    }
    else
        return false;
}


bool CScript::InterpreteKinovaCurrentPosition(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_CURRENT_POSITION_STRUCT")){

        if(_line.contains("x")){
            int equal_index = _line.indexOf("=");

            if(equal_index >= 0){
                _step_info.manipulation_option.kinova_force_option.str_result_variable =
                        _line.mid(0, equal_index - 1).trimmed();
            }
            return true;
        }
        if(_line.contains("y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_current_position.str_y = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_current_position.str_z = _line.mid(colone_index + 1).trimmed();
            return true;
        }

    }
    else if(_line.contains("KINOVA_CURRENT_POSITION_FUNCTION")){
        _step_info.function_index = MP_KINOVA_CURRENT_POSITION;
    }
    else
        return false;


    return true;
}

bool CScript::InterpreteLRFVHorizenCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_V_HORIZEN_CTRL_STRUCT")){

        if(_line.contains("mode")){
            int colone_index = _line.indexOf("=");
            if(_line.mid(colone_index + 1).trimmed().toInt() == 1){
                _step_info.manipulation_option.lrf_v_h_ctrl_struct.lrf_info_struct.mode = L_M_ROUGH;
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 2){
                _step_info.manipulation_option.lrf_v_h_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_LEFT | L_M_VALUE_RANSAC);
            }
            if(_line.mid(colone_index + 1).trimmed().toInt() == 3){
                _step_info.manipulation_option.lrf_v_h_ctrl_struct.lrf_info_struct.mode = (L_M_PRECISE | L_M_DIR_RIGHT | L_M_VALUE_RANSAC);
            }
            return true;
        }

        if(_line.contains("desired_h_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_h_ctrl_struct.desired_h_location = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_h_ctrl_struct.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_h_ctrl_struct.lrf_info_struct
                    .s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_h_ctrl_struct.lrf_info_struct
                    .e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_h_ctrl_struct.lrf_info_struct
                    .inlier_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_h_ctrl_struct.loop_sleep = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }
        if(_line.contains("velocity")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_v_h_ctrl_struct.velocity = _line.mid(colone_index + 1).trimmed().toInt();
            return true;
        }

    }
    else if(_line.contains("LRF_V_HORIZEN_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_V_HORIZEN_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFKinovaVerticalCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_KINOVA_VERTICAL_CTRL_STRUCT")){

        if(_line.contains("desired_distance")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_vertical_option.desired_distance = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_vertical_option.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_vertical_option.s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_vertical_option.e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_vertical_option.inlier_lrf_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_vertical_option.loop_sleep = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("sensor_option")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_kinova_vertical_option.sensor_option = true;
            else
                _step_info.manipulation_option.lrf_kinova_vertical_option.sensor_option = false;

            return true;
        }
    }
    else if(_line.contains("LRF_KINOVA_VERTICAL_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_KINOVA_VERTIVAL_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFKinovaHorizenCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("LRF_KINOVA_HORIZEN_CTRL_STRUCT")){

        if(_line.contains("desired_inlier_deg_avr")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.desired_inlier_deg_avr = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("wrench_hanger_index")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.wrench_hanger_index_str = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("wrench_location_deg_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.wrench_location_deg_1 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_deg_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.wrench_location_deg_2 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_deg_3")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.wrench_location_deg_3 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_deg_4")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.wrench_location_deg_4 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_deg_5")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.wrench_location_deg_5 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_location_deg_6")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.wrench_location_deg_6 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.inlier_lrf_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_horizen_option.loop_sleep = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("sensor_option")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_kinova_horizen_option.sensor_option = true;
            else
                _step_info.manipulation_option.lrf_kinova_horizen_option.sensor_option = false;

            return true;
        }

    }
    else if(_line.contains("LRF_KINOVA_HORIZEN_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_KINOVA_HORIZEN_CONTROL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteLRFKinovaWrenchLocationCtrl(QString _line, STEP_INFO& _step_info){


    if(_line.contains("LRF_KINOVA_WRENCH_LOCATION_CTRL_STRUCT")){

        if(_line.contains("desired_start_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.desired_start_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("desired_end_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.desired_end_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("wrench_hanger_index")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.wrench_hanger_index_str = _line.mid(colone_index + 1).trimmed();
            return true;
        }

        if(_line.contains("wrench_rel_location_1")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.wrench_rel_location_1 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_rel_location_2")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.wrench_rel_location_2 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_rel_location_3")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.wrench_rel_location_3 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_rel_location_4")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.wrench_rel_location_4 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_rel_location_5")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.wrench_rel_location_5 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("wrench_rel_location_6")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.wrench_rel_location_6 = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("error")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.error = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("s_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.s_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("e_deg")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.e_deg = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("inlier_lrf_dst")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.inlier_lrf_dst = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("loop_sleep")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.lrf_kinova_wrench_option.loop_sleep = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("sensor_option")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.lrf_kinova_wrench_option.sensor_option = true;
            else
                _step_info.manipulation_option.lrf_kinova_wrench_option.sensor_option = false;

            return true;
        }

    }
    else if(_line.contains("LRF_KINOVA_WRENCH_LOCATION_CTRL_FUNCTION")){
        _step_info.function_index = MP_LRF_KINOVA_WRENCH_LOVATION_CTRL;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteKinovaRotateValveCtrl(QString _line, STEP_INFO& _step_info){

    if(_line.contains("KINOVA_ROTATE_VALVE_STRUCT")){

        if(_line.contains("center_x")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.center_x = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("center_y")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.center_y = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("center_z")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.center_z = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("theta")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.theta = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }


        if(_line.contains("radius_16mm")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius_16mm = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("radius_17mm")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius_17mm = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("radius_18mm")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius_18mm = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("radius_19mm")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius_19mm = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("radius_22mm")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius_22mm = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }
        if(_line.contains("radius_24mm")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius_24mm = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("radius_offset_mm")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius_offset_mm = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("radius")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.radius = _line.mid(colone_index + 1).trimmed().toDouble();
            return true;
        }

        if(_line.contains("wrench_size")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.str_wrench_size = _line.mid(colone_index + 1).trimmed();
            return true;
        }
        if(_line.contains("valve_rotation_angle")){
            int colone_index = _line.indexOf("=");
            _step_info.manipulation_option.kinova_rotate_valve_option.str_valve_rotation_angle = _line.mid(colone_index + 1).trimmed();
            return true;
        }

        if(_line.contains("using_current_coord")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.kinova_rotate_valve_option.using_current_coord = true;
            else
                _step_info.manipulation_option.kinova_rotate_valve_option.using_current_coord = false;

            return true;
        }

        if(_line.contains("init_angle")){
            int colone_index = _line.indexOf("=");

            QString str_option;
            str_option = _line.mid(colone_index + 1).trimmed();

            if(str_option.contains("true"))
                _step_info.manipulation_option.kinova_rotate_valve_option.init_angle = true;
            else
                _step_info.manipulation_option.kinova_rotate_valve_option.init_angle = false;

            return true;
        }

    }
    else if(_line.contains("KINOVA_ROTATE_VALVE_FUNCTION")){
        _step_info.function_index = MP_KINOVA_ROTATE_VALVE;
    }
    else
        return false;

    return true;
}

bool CScript::InterpreteConditionallyIterate(QString _line, STEP_INFO& _step_info){

    if(_line.contains("CONDITIONALLY_ITERATION(")){

        int o_bracket = _line.indexOf("(");//open  bracket
        int c_bracket = _line.indexOf(")");//close bracket

        if(o_bracket >=0){
            _step_info.iterate_option.str_condition =
                    _line.mid(o_bracket + 1, c_bracket - o_bracket - 1).trimmed();

            _step_info.function_index = GR_CONDITIONALLY_ITERATION;
            return true;
        }
        else
            return false;
    }

    if(_line.contains("str_mission_name")){
        int equal_index = _line.indexOf("=");

        QString str_option;
        str_option = _line.mid(equal_index + 1).trimmed();

        _step_info.iterate_option.str_mission_name = str_option;
    }

    if(_line.contains("start_step_index")){
        int equal_index = _line.indexOf("=");
        _step_info.iterate_option.start_step_index = _line.mid(equal_index + 1).trimmed().toInt();
    }

    if(_line.contains("end_step_index")){
        int equal_index = _line.indexOf("=");
        _step_info.iterate_option.end_step_index = _line.mid(equal_index + 1).trimmed().toInt();
    }


    return true;
}

bool CScript::SetIntVariable(QString _str_variable, MISSION_SCRIPT& _mission_script, int _value){

    //Check Local Int Value
    vector<SCRIPT_INT>::iterator iter;
    for( iter = _mission_script.vec_lc_int.begin();
         iter != _mission_script.vec_lc_int.end();
         iter++){

        if((*iter).variable_name == _str_variable){
            (*iter).int_value = _value;
            return true;
        }
    }
    //Check Global Int Value
    for( iter = mvec_global_int.begin();
         iter != mvec_global_int.end();
         iter++){

        if((*iter).variable_name == _str_variable){
            (*iter).int_value = _value;
            return true;
        }
    }

    return false;
}

bool CScript::SetDoubleVariable(QString _str_variable, MISSION_SCRIPT& _mission_script, double _value){

    //Check Local Int Value
    vector<SCRIPT_DOUBLE>::iterator iter;
    for( iter = _mission_script.vec_lc_double.begin();
         iter != _mission_script.vec_lc_double.end();
         iter++){

        if((*iter).variable_name == _str_variable){
            (*iter).double_value = _value;
            return true;
        }
    }
    //Check Global Int Value
    for( iter = mvec_global_double.begin();
         iter != mvec_global_double.end();
         iter++){

        if((*iter).variable_name == _str_variable){
            (*iter).double_value = _value;
            return true;
        }
    }

    return false;
}

bool CScript::SetBoolVariable(QString _str_variable, MISSION_SCRIPT& _mission_script, bool _result){

    //Check Local Bool Value
    vector<SCRIPT_BOOL>::iterator iter;
    for( iter = _mission_script.vec_lc_bool.begin();
         iter != _mission_script.vec_lc_bool.end();
         iter++){

        if((*iter).variable_name == _str_variable){
            (*iter).bool_value = _result;
            return true;
        }
    }
    //Check Global Bool Value
    for( iter = mvec_global_bool.begin();
         iter != mvec_global_bool.end();
         iter++){

        if((*iter).variable_name == _str_variable){
            (*iter).bool_value = _result;
            return true;
        }
    }

    return false;
}

int CScript::InterpreteIntVariable(QString _line, MISSION_SCRIPT _mission_script/*For Local Variable*/){

    QString str_option;
    str_option = _line;

    QRegExp re("\\d*");
    if (re.exactMatch(str_option)){
        int rst =  _line.toInt();
        return rst;
    }

    //Check Local Bool Value
    vector<SCRIPT_INT>::iterator iter;
    for( iter = _mission_script.vec_lc_int.begin();
         iter != _mission_script.vec_lc_int.end();
         iter++){

        int compare = QString::compare((*iter).variable_name, str_option, Qt::CaseSensitive);

        if(compare == 0){
            return (*iter).int_value;
        }
    }
    //Check Global Bool Value
    for( iter = mvec_global_int.begin();
         iter != mvec_global_int.end();
         iter++){

        int compare = QString::compare((*iter).variable_name, str_option, Qt::CaseSensitive);

        if(compare == 0){
            return (*iter).int_value;
        }
    }

    return UNDEFINE_VARIABLE;
}

bool CScript::InterpreteBoolVariable(QString _line, MISSION_SCRIPT _mission_script/*For Local Variable*/){

    bool fl_inverse_index = false;
    int inverse_index = _line.indexOf("!");

    if(inverse_index >= 0)
        fl_inverse_index = true;

    QString str_option;
    str_option = _line.mid(inverse_index + 1).trimmed();

    if(str_option.contains("true") && (str_option.size() == 4)){
        if(fl_inverse_index)
            return false;
        else if(!fl_inverse_index)
            return true;
    }

    else if(str_option.contains("false") && (str_option.size() == 5)){
        if(fl_inverse_index)
            return true;
        else if(!fl_inverse_index)
            return false;
    }

    else{
        //Check Local Int Value
        vector<SCRIPT_BOOL>::iterator iter;
        for( iter = _mission_script.vec_lc_bool.begin();
             iter != _mission_script.vec_lc_bool.end();
             iter++){

            int compare = QString::compare((*iter).variable_name, str_option, Qt::CaseSensitive);

            if(compare == 0){
                if(fl_inverse_index)
                    return !(*iter).bool_value;
                else if(!fl_inverse_index)
                    return (*iter).bool_value;
            }
        }
        //Check Global Int Value
        for( iter = mvec_global_bool.begin();
             iter != mvec_global_bool.end();
             iter++){

            int compare = QString::compare((*iter).variable_name, str_option, Qt::CaseSensitive);

            if(compare == 0){
                if(fl_inverse_index)
                    return !(*iter).bool_value;
                else if(!fl_inverse_index)
                    return (*iter).bool_value;
            }
        }
    }

    return false;
}

bool CScript::InterpreteSignOfInequality(bool &_fl_result, QString _line, MISSION_SCRIPT _mission_script/*For Local Variable*/){

    QString str_variable;
    QString str_value;

    int variable_int = 0;
    double variable_double = 0;

    int value_int = 0;
    double value_double = 0;

    int sign_of_inequality_1 = 0;//==
    int sign_of_inequality_2 = 0;//>
    int sign_of_inequality_3 = 0;//<
    int sign_of_inequality_4 = 0;//!=

    sign_of_inequality_1 = _line.indexOf("==");
    sign_of_inequality_2 = _line.indexOf(">");
    sign_of_inequality_3 = _line.indexOf("<");
    sign_of_inequality_4 = _line.indexOf("!=");

    if(sign_of_inequality_1 >= 0){//==

        str_value = _line.mid(sign_of_inequality_1 + 2/*Because, != is 2 character*/);
        str_variable = _line.mid(0, sign_of_inequality_1 - 1);

        variable_int = InterpreteIntVariable(str_variable,_mission_script);
        variable_double = InterpreteDoubleVariable(str_variable,_mission_script);

        if(variable_int != UNDEFINE_VARIABLE){
            value_int = str_value.toInt();

            if(variable_int == value_int){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
        else if(variable_double != UNDEFINE_VARIABLE){
            value_double = str_value.toDouble();

            if(variable_double == value_double){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
    }
    else if(sign_of_inequality_2 >= 0){// >

        str_value = _line.mid(sign_of_inequality_2 + 1);
        str_variable = _line.mid(0, sign_of_inequality_2 - 1);

        variable_int = InterpreteIntVariable(str_variable,_mission_script);
        variable_double = InterpreteDoubleVariable(str_variable,_mission_script);

        if(variable_int != UNDEFINE_VARIABLE){
            value_int = str_value.toInt();

            if(variable_int > value_int){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
        else if(variable_double != UNDEFINE_VARIABLE){
            value_double = str_value.toDouble();

            if(variable_double > value_double){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
    }
    else if(sign_of_inequality_3 >= 0){// <

        str_value = _line.mid(sign_of_inequality_3 + 1);
        str_variable = _line.mid(0, sign_of_inequality_3 - 1);

        variable_int = InterpreteIntVariable(str_variable,_mission_script);
        variable_double = InterpreteDoubleVariable(str_variable,_mission_script);

        if(variable_int != UNDEFINE_VARIABLE){
            value_int = str_value.toInt();

            if(variable_int < value_int){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
        else if(variable_double != UNDEFINE_VARIABLE){
            value_double = str_value.toDouble();

            if(variable_double < value_double){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
    }
    else if(sign_of_inequality_4 >= 0){// !=

        str_value = _line.mid(sign_of_inequality_4 + 2/*Because, != is 2 character*/);
        str_variable = _line.mid(0, sign_of_inequality_4 - 1);

        variable_int = InterpreteIntVariable(str_variable,_mission_script);
        variable_double = InterpreteDoubleVariable(str_variable,_mission_script);

        if(variable_int != UNDEFINE_VARIABLE){
            value_int = str_value.toInt();

            if(variable_int != value_int){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
        else if(variable_double != UNDEFINE_VARIABLE){
            value_double = str_value.toDouble();

            if(variable_double != value_double){
                _fl_result = true;
                return true;
            }
            else{
                _fl_result = false;
                return true;
            }
        }
    }

    return false;
}

double CScript::InterpreteDoubleVariable(QString _line, MISSION_SCRIPT _mission_script/*For Local Variable*/){

    QString str_option;
    str_option = _line;

//    QRegExp re("\\d*");
//    if (re.exactMatch(str_option)){
//        double rst =  _line.toDouble();
//        return rst;
//    }

    //Check Local Double Value
    vector<SCRIPT_DOUBLE>::iterator iter;
    for( iter = _mission_script.vec_lc_double.begin();
         iter != _mission_script.vec_lc_double.end();
         iter++){

        int compare = QString::compare((*iter).variable_name, str_option, Qt::CaseSensitive);

        if(compare == 0){
            return (*iter).double_value;
        }
    }
    //Check Global Double Value
    for( iter = mvec_global_double.begin();
         iter != mvec_global_double.end();
         iter++){

        int compare = QString::compare((*iter).variable_name, str_option, Qt::CaseSensitive);

        if(compare == 0){
            return (*iter).double_value;
        }
    }

    //else double number
    double rst =  _line.toDouble();

    return rst;

//    return UNDEFINE_VARIABLE;
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

    QString msessage;

    QString mission_font_option_head = "<font color=red>";
    QString mission_font_option_tail = "</font>";

    QString step_font_option_head = "<font color=blue>";
    QString step_font_option_tail = "</font>";

    if(mpary_mission_script[0].step_vecor.empty()){
        msessage = mission_font_option_head;
        msessage += "Check Mission Status";
        msessage += mission_font_option_tail;
        emit SignalScriptMessage(msessage);
        return false;
    }

    unsigned int step_start_inx = mstruct_player_option.start_step_num;
    unsigned int step_end___inx = mstruct_player_option.end_step_num;

    unsigned int mission_start_inx = mstruct_player_option.start_mission_num < 0 ? 0 : mstruct_player_option.start_mission_num;
    unsigned int mission_end___inx = mstruct_player_option.end_mission_num < 0 ? 0 : mstruct_player_option.end_mission_num;

    if(mission_end___inx > (mstruct_scenario.mission_file_name.size() -1))
            mission_end___inx = (mstruct_scenario.mission_file_name.size() -1);

    for(unsigned int i = mission_start_inx; i <= mission_end___inx; i++){

        if(step_end___inx > (mpary_mission_script[i].step_vecor.size() -1))
                step_end___inx = (mpary_mission_script[i].step_vecor.size() -1);

        msessage = mission_font_option_head;
        msessage += "Start Mission: " + mpary_mission_script[i].mission_title;
        msessage += mission_font_option_tail;

        emit SignalScriptMessage(msessage);

        for(unsigned int j = step_start_inx; j <= step_end___inx; j++){

            while(IsMissionPaused()){
                usleep(500);
            }

            if(IsMissionTerminated()){
                i = mission_end___inx + 1;
                j = step_end___inx + 1;
                return false;
            }

            msessage = step_font_option_head;
            msessage += "Start Step: " + QString::number(j) + mpary_mission_script[i].step_vecor.at(j).step_title;
            msessage += step_font_option_tail;

            emit SignalScriptMessage(msessage);

            if(mpary_mission_script[i].step_vecor.at(j).before_sleep != 0)
                msleep(mpary_mission_script[i].step_vecor.at(j).before_sleep);

            if(mpary_mission_script[i].step_vecor.at(j).fl_condition_if_flag){
                bool fl_condition = false;

                if(
                        mpary_mission_script[i].step_vecor.at(j).str_if.contains("==") ||
                        mpary_mission_script[i].step_vecor.at(j).str_if.contains(">") ||
                        mpary_mission_script[i].step_vecor.at(j).str_if.contains("<") ||
                        mpary_mission_script[i].step_vecor.at(j).str_if.contains("!=")
                        ){
                    if(!InterpreteSignOfInequality(fl_condition, mpary_mission_script[i].step_vecor.at(j).str_if, mpary_mission_script[i])){
                        std::cout << "Fail to Interprete Sign of Inequality!!" << std::endl;
                        continue;
                    }
                }

                else
                    fl_condition = InterpreteBoolVariable(mpary_mission_script[i].step_vecor.at(j).str_if, mpary_mission_script[i]);

                if(!fl_condition){
                    if(mpary_mission_script[i].step_vecor.at(j).fl_condition_else_flag){

                        QString str_index = mpary_mission_script[i].step_vecor.at(j).str_else;

                        int colone_index = str_index.indexOf(":");
                        int step_number = str_index.mid(colone_index + 1).toInt();

                        if((unsigned int)step_number > step_end___inx){
                            step_number = step_end___inx;
                        }
                        if(step_number < 0){
                            step_number = 0;
                        }

                        j = step_number - 1;

                        msessage = "Jump to Step " + QString::number(step_number);
                        emit SignalScriptMessage(msessage);
                    }
                    continue;
                }
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_ROTATOR){

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.rotator_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_ROTATOR);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_K_VEHICLE_HORIZEN_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_vehicle_horizen_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_K_VEHICLE_HORIZEN_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_K_VEHICLE_ANGLE_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_vehicle_angle_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_K_VEHICLE_ANGLE_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_PARKING_RETRY){

                PARKING_RETRY_STRUCT parking_option;

                QString str_bias;
                str_bias = mpary_mission_script[i].step_vecor.at(j).driving_option.parking_retry_option.str_bias;

                parking_option.bias = InterpreteDoubleVariable(str_bias, mpary_mission_script[i]);

//                mpc_drivig->SetManipulationOption(parking_option);
                mpc_drivig->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).driving_option.parking_retry_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_PARKING_RETRY);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_WRENCH_RECOG){

                QString str_index;
                str_index = mpary_mission_script[i].step_vecor.at(j).driving_option.wrench_recog_option.str_desired_wrench_index;

                mpary_mission_script[i].step_vecor.at(j).driving_option.wrench_recog_option.desired_wrench_index =
                        InterpreteIntVariable(str_index, mpary_mission_script[i]);

                mpc_drivig->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).driving_option.wrench_recog_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_WRENCH_RECOG);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_VEHICLE_DRIVE_TO_PANEL){
                mpc_drivig->SetDrivingOption(mpary_mission_script[i].step_vecor.at(j).driving_option.driving_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_DRIVE_TO_PANEL);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_LOCALIZATION){

                double body_x = 0;
                double body_y = 0;

                mpc_manipulation->GetKinovaBodyPose(body_x, body_y);
                body_x *= 0.001;
                body_y *= 0.001;

                mpary_mission_script[i].step_vecor.at(j).driving_option.vehicle_localization_option.desired_h_dst -= body_y;

                mpc_drivig->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).driving_option.vehicle_localization_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_LOCAL_ON_PANE);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == DR_VEHICLE_PARKING){
                mpc_drivig->SetParkingOption(mpary_mission_script[i].step_vecor.at(j).driving_option.parking_option);
                mpc_drivig->SelectMainFunction(DRIVE_INX_PARKING__PANEL);

                while(mpc_drivig->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_ROTATE_VALVE){

                int wrench_size = 0;
                QString str_wrench_size;
                str_wrench_size = mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_rotate_valve_option.str_wrench_size;

                wrench_size = InterpreteIntVariable(str_wrench_size, mpary_mission_script[i]);
                mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_rotate_valve_option.wrench_size = wrench_size;

                int valve_rotation = 0;
                QString str_valve_rotation;
                str_valve_rotation = mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_rotate_valve_option.str_valve_rotation_angle;

                valve_rotation = InterpreteDoubleVariable(str_valve_rotation, mpary_mission_script[i]);
                mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_rotate_valve_option.valve_rotation_angle = valve_rotation;


                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_rotate_valve_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_ROTATE_VALVE);

                while(mpc_manipulation->isRunning());
            }

            /*New LRF Kinova - Vehicle Control*/
            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_K_VERTIVAL_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_v_ctrl_struct);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_K_VERTIVAL_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_K_HORIZEN_CONTROL){
                int wrench_index = 0;
                QString str_index;
                str_index = mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_h_ctrl_struct.wrench_hanger_index_str;

                wrench_index = InterpreteIntVariable(str_index, mpary_mission_script[i]);

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_h_ctrl_struct.wrench_hanger_index =
                        wrench_index;

                QString str_current_y;
                double kinova_current_y;

                str_current_y = mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_h_ctrl_struct.str_kinova_current_y;

                kinova_current_y = InterpreteDoubleVariable(str_current_y, mpary_mission_script[i]);

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_h_ctrl_struct.kinova_current_y =
                        kinova_current_y;

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_h_ctrl_struct);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_K_HORIZEN_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_K_ANGLE_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_k_a_ctrl_struct);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_K_ANGLE_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_V_ANGLE_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_v_a_ctrl_struct);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_V_ANGLE_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_WRENCH_SEARCHING){

                QString str_index;
                str_index = mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_searching_option.str_desired_wrench_index;

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_searching_option.desired_wrench_index =
                        InterpreteIntVariable(str_index, mpary_mission_script[i]);

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_searching_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_WRENCH_SEARCHING);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_CURRENT_POSITION){

                SetKinovaAxisValue(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_current_position,
                                    mpary_mission_script[i]);

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_current_position);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_POSITION);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_V_HORIZEN_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_v_h_ctrl_struct);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_V_HORIZEN_CTRL);

                while(mpc_manipulation->isRunning());
            }
            //-------------------------------------------------------------------------------------------------------------------------------------------

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_KINOVA_VERTIVAL_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_vertical_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_KINOVA_HORIZEN_CONTROL){

                int wrench_index = 0;
                QString str_index;
                str_index = mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_horizen_option.wrench_hanger_index_str;

                wrench_index = InterpreteIntVariable(str_index, mpary_mission_script[i]);

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_horizen_option.wrench_hanger_index =
                        wrench_index;

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_horizen_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_KINOVA_HORIZEN_CTRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_LRF_KINOVA_WRENCH_LOVATION_CTRL){

                int wrench_index = 0;
                QString str_index;
                str_index = mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_wrench_option.wrench_hanger_index_str;

                wrench_index = InterpreteIntVariable(str_index, mpary_mission_script[i]);

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_wrench_option.wrench_hanger_index =
                        wrench_index;

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.lrf_kinova_wrench_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_KINOVA_WRENCH_LOCATION);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_FORCE_CONTROL){

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_force_option.fl_kinova_force_ctrl_sensing_option = false;

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_force_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_FORCE_CLRL);

                while(mpc_manipulation->isRunning());

                bool function_result = false;
                QString variable_name = mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_force_option.str_result_variable;

                function_result = mpc_manipulation->GetMainFunctionResult();
                SetBoolVariable(variable_name,mpary_mission_script[i],function_result);
                mpc_manipulation->SetMainFunctionResult(false);
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_FORCE_CHECK){

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_force_check_option.fl_kinova_force_sensing_option = false;

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_force_check_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_FORCE_CHECK);

                while(mpc_manipulation->isRunning());

                bool function_result = false;
                QString variable_name = mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_force_check_option.str_result_variable;

                function_result = mpc_manipulation->GetMainFunctionResult();
                SetBoolVariable(variable_name,mpary_mission_script[i],function_result);
                mpc_manipulation->SetMainFunctionResult(false);
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_GRIPPER_FORCE_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_force_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_FORCE_CLRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_GRIPPER_GO_TO_REL_POSE){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.grippper_go_to_rel_pose_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_GO_TO_REL_POSE);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_GRIPPER_MAGNET_CONTROL){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_magnet_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_MAGNET_CLRL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_GRIPPER_FIND_VALVE_LOCATION){

                int valve_size = 0;
                QString str_valve_size;
                str_valve_size = mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_location_option.str_valve_size;

                valve_size = InterpreteIntVariable(str_valve_size, mpary_mission_script[i]);
                mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_location_option.valve_size = valve_size;

                double valve_rotation = 0;
                QString str_valve_rotation;
                str_valve_rotation = mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_location_option.str_valve_rotation;

                valve_rotation = InterpreteDoubleVariable(str_valve_rotation, mpary_mission_script[i]);
                mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_location_option.valve_rotation = valve_rotation;


                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_location_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_FIND_VALVE_LOCATION);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_GRIPPER_VALVE_SIZE_RECOG){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_recog_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_VALVE_SIZE_RECOG);

                while(mpc_manipulation->isRunning());

                int valve_size = mpc_manipulation->GetGripperKinovaValveRecogOption().valve_size;
                double valve_rotation_angle = mpc_manipulation->GetValveRotationRecogResult();

                QString size_variable_name = mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_recog_option.str_size_result_variable;
                QString rotation_variable_name = mpary_mission_script[i].step_vecor.at(j).manipulation_option.gripper_kinova_valve_recog_option.str_rotation_result_variable;

                SetIntVariable(size_variable_name, mpary_mission_script[i], valve_size);
                SetDoubleVariable(rotation_variable_name, mpary_mission_script[i], valve_rotation_angle);

                mpc_manipulation->SetValveSizeRecogResult(0);
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_LRF_CHECK_V_DST){

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.check_v_distance_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_LRF_CHECK_V_DST);

                while(mpc_manipulation->isRunning());

                bool check_result = mpc_manipulation->GetCheckVDstOption().result;
                double bias = mpc_manipulation->GetCheckVDstOption().biase;

                QString str_check_result = mpary_mission_script[i].step_vecor.at(j).manipulation_option.check_v_distance_option.str_result;
                QString str_bias = mpary_mission_script[i].step_vecor.at(j).manipulation_option.check_v_distance_option.str_bias;

                SetBoolVariable(str_check_result, mpary_mission_script[i], check_result);
                SetDoubleVariable(str_bias, mpary_mission_script[i], bias);

                CHECK_CURRENT_V_DISTANCE_STRUCT check_v_dst;

                check_v_dst.biase = 0;
                check_v_dst.fl_check_v_dst = false;

                mpc_manipulation->SetManipulationOption(check_v_dst);
            }


            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_ALIGN_TO_PANEL){

                if(mpary_mission_script[i].step_vecor.at(j).fl_do_init_motion){
                    mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_INIT_MOTION);
                    while(mpc_manipulation->isRunning());
                }

                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_ALIGN_TO_PANEL);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_MANIPULATE){
                SetKinovaAxisValue(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_manipulate_option);

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_manipulate_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_MANIPULATE);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_FIT_TO_VALVE_POSE){

                int valve_size = 0;
                double rotation_angle = 0;

                QString str_valve_size = 0;
                QString str_rotation_angle = 0;
                str_valve_size =
                        mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_fit_to_valve_pose.str_valve_size;
                str_rotation_angle =
                        mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_fit_to_valve_pose.str_rotation_angle;

                valve_size = InterpreteIntVariable(str_valve_size, mpary_mission_script[i]);
                rotation_angle = InterpreteDoubleVariable(str_rotation_angle, mpary_mission_script[i]);

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_fit_to_valve_pose.valve_size
                        = valve_size;
                mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_fit_to_valve_pose.valve_rotation_angle
                        = rotation_angle;

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_fit_to_valve_pose);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_FIT_TO_VALVE);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_KINOVA_LRF_VALVE_SEARCHING){
                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.kinova_lrf_valve_searching_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_LRF_VALVE_SEARCHING);

                while(mpc_manipulation->isRunning());
            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == MP_WRENCH_RECOGNITION){

                int valve_size = InterpreteIntVariable(mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_recognition_option.str_valve_size,
                                                        mpary_mission_script[i]);
                int num_of_wrench = InterpreteIntVariable(mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_recognition_option.str_num_of_wrench,
                                                        mpary_mission_script[i]);

                mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_recognition_option.valve_size = valve_size;
                mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_recognition_option.num_of_wrench = num_of_wrench;

                mpc_manipulation->SetManipulationOption(mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_recognition_option);
                mpc_manipulation->SelectMainFunction(MANIPUL_INX_WRENCH_RECOGNITION);

                while(mpc_manipulation->isRunning());

                WRENCH_RECOGNITION wrech_recognitino_rst = mpc_manipulation->GetWrenchRecognitionOption();
                int wrench_location = wrech_recognitino_rst.wrench_location;// 1 ~ 6

                QString variable_name = mpary_mission_script[i].step_vecor.at(j).manipulation_option.wrench_recognition_option.str_result_variable;
                SetIntVariable(variable_name, mpary_mission_script[i], wrench_location);

            }

            if(mpary_mission_script[i].step_vecor.at(j).function_index == GR_CONDITIONALLY_ITERATION){

                bool fl_condition = false;

                fl_condition = InterpreteBoolVariable(mpary_mission_script[i].step_vecor.at(j).iterate_option.str_condition, mpary_mission_script[i]);

                if(fl_condition){
                    j = mpary_mission_script[i].step_vecor.at(j).iterate_option.start_step_index;
                    continue;
                }
            }

            if(mpary_mission_script[i].step_vecor.at(j).after__sleep != 0)
                msleep(mpary_mission_script[i].step_vecor.at(j).after__sleep);

            msessage ="End Step: " + mpary_mission_script[i].step_vecor.at(j).step_title;
            emit SignalScriptMessage(msessage);
        }

        msessage = mission_font_option_head;
        msessage += "End Mission: " + mpary_mission_script[i].mission_title;
        msessage += mission_font_option_tail;

        step_end___inx = 100; //Dummy
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
