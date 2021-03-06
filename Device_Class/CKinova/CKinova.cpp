#include "CKinova.h"

CKinova::CKinova(){

    //We load the library
    mp_commandLayer_handle= dlopen("/opt/kinova/GUI/Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

    //We load the functions from the library (Under Windows, use GetProcAddress)
    Kinova_InitAPI = (int (*)()) dlsym(mp_commandLayer_handle,"InitAPI");
    Kinova_CloseAPI = (int (*)()) dlsym(mp_commandLayer_handle,"CloseAPI");
    Kinova_MoveHome = (int (*)()) dlsym(mp_commandLayer_handle,"MoveHome");
    Kinova_SetFrameType = (int (*)(int)) dlsym(mp_commandLayer_handle,"SetFrameType");

    Kinova_SetTorqueZero = (int (*)(int)) dlsym(mp_commandLayer_handle,"SetTorqueZero");
    Kinova_SetTorqueGain = (int (*)(int, int)) dlsym(mp_commandLayer_handle,"SetTorqueGain");;

    Kinova_RestoreFactoryDefault = (int (*)()) dlsym(mp_commandLayer_handle,"RestoreFactoryDefault");;

//    restoreFactoryDefault )()


    //Kinova_InitFingers = (int (*)()) dlsym(mp_commandLayer_handle,"InitFingers");
    Kinova_GetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(mp_commandLayer_handle,"GetDevices");
    Kinova_SetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(mp_commandLayer_handle,"SetActiveDevice");
    Kinova_SendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(mp_commandLayer_handle,"SendBasicTrajectory");
    Kinova_GetQuickStatus = (int (*)(QuickStatus &)) dlsym(mp_commandLayer_handle,"GetQuickStatus");
    Kinova_GetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(mp_commandLayer_handle,"GetCartesianCommand");
    Kinova_SetTorqueCommandMax = (int (*)(float Command[COMMAND_SIZE])) dlsym(mp_commandLayer_handle, "SetTorqueCommandMax");
    Kinova_SendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(mp_commandLayer_handle,"SendAdvanceTrajectory");
    Kinova_EraseAllTrajectories = (int (*)()) dlsym(mp_commandLayer_handle,"EraseAllTrajectories");
    Kinova_GetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(mp_commandLayer_handle,"GetCartesianPosition");

    KinovaUnitStepMoving = (int (*)(TrajectoryPoint)) dlsym(mp_commandLayer_handle,"SendAdvanceTrajectory");

    Kinova_SendJoystickCommand = (int (*)(JoystickCommand command)) dlsym(mp_commandLayer_handle,"SendJoystickCommand");
    Kinova_GetJoystickValue = (int (*)(JoystickCommand &command)) dlsym(mp_commandLayer_handle,"GetJoystickValue");

    Kinova_GetForcesInfo = (int (*)(ForcesInfo &)) dlsym(mp_commandLayer_handle,"GetForcesInfo");
    Kinova_GetCartesianForce = (int (*)(CartesianPosition &)) dlsym(mp_commandLayer_handle, "GetCartesianForce");

    fl_kinova_init = false;
    fl_kinova_manipulation = false;
    fl_kinova_init_position = false;
    fl_kinova_get_force_feedback = false;

    theta = 0.0;
    initialAngle = 0.0;
//    m_base_frame_rot = -5.0;
}

CKinova::~CKinova(){
    if(fl_kinova_init)
        Kinova_CloseAPI();
}

bool CKinova::InitKinova(){

    if((Kinova_InitAPI == NULL) || (Kinova_CloseAPI == NULL) ||
            (Kinova_GetQuickStatus == NULL) || (Kinova_SendBasicTrajectory == NULL) ||
            (Kinova_SendBasicTrajectory == NULL) || (Kinova_MoveHome == NULL) /*|| (Kinova_InitFingers == NULL)*/ ||
            (Kinova_GetCartesianCommand == NULL) /*|| (Kinova_SetTorqueCommandMax == NULL)*/
            || (Kinova_SendAdvanceTrajectory == NULL) ||
            (Kinova_EraseAllTrajectories == NULL) || (Kinova_GetCartesianPosition == NULL) ||
            (Kinova_GetForcesInfo == NULL) || (Kinova_SendJoystickCommand == NULL) || (Kinova_SetFrameType == NULL) //|| (Kinova_GetJoystickValue == NULL)
//            || (Kinova_SetTorqueZero == NULL) || (Kinova_SetTorqueGain == NULL)
            ){
        if(Kinova_InitAPI == NULL){
            printf("Kinova Init API");
        }
        if(Kinova_CloseAPI == NULL){
            printf("Kinova Close API");
        }
        if(Kinova_GetQuickStatus == NULL){
            printf("Kinova Get Quick Status");
        }
        if(Kinova_SendBasicTrajectory == NULL){
            printf("Kinova Send Basic Trajectory");
        }
        if(Kinova_MoveHome == NULL){
            printf("Kinova Move Home");
        }
//        if(Kinova_InitFingers == NULL){
//            printf("Kinova_InitFingers");
//        }
        if(Kinova_GetCartesianCommand == NULL){
            printf("Kinova_GetCartesianCommand");
        }
        if(Kinova_SetTorqueCommandMax == NULL){
            printf("Kinova_SetTorqueCommandMax");
        }
        if(Kinova_SendAdvanceTrajectory == NULL){
            printf("Kinova_SendAdvanceTrajectory");
        }
        if(Kinova_EraseAllTrajectories == NULL){
            printf("Kinova_EraseAllTrajectories");
        }
        if(Kinova_GetCartesianPosition == NULL){
            printf("Kinova_GetCartesianPosition");
        }
        if(Kinova_GetForcesInfo == NULL){
            printf("Kinova_GetForcesInfo");
        }
        if(Kinova_SendJoystickCommand == NULL){
            printf("Kinova_SendJoystickCommand");
        }
        if(Kinova_SetFrameType == NULL){
            printf("Kinova_SetFrameType");
        }
        if(Kinova_GetJoystickValue == NULL){
            printf("Kinova_GetJoystickValue");
        }

        return false;
    }

    else{
        int result = (*Kinova_InitAPI)();

        if(result != 1){
            Kinova_CloseAPI();
            return false;
        }

        m_kinova_devicesCount = Kinova_GetDevices(m_kinova_list, result);

        //We use only one kinova arm, So list number is always '0'.
        Kinova_SetActiveDevice(m_kinova_list[0]);

        //MICO-2 = 1
        //JACO-2 = 3
        if(m_kinova_list[0].DeviceType != 1){
            Kinova_CloseAPI();
            return false;
        }

        Kinova_SetFrameType(0);

        fl_kinova_init = true;
        return true;
    }
}

void CKinova::CloseKinova(){

    Kinova_CloseAPI();

    fl_kinova_init = false;
}

bool CKinova::IsKinovaInitialized(){
    return fl_kinova_init;
}

bool CKinova::IsKinovaInitPosition(){
    return fl_kinova_init_position;
}

void CKinova::KinovaInitMotion(){
    if(!fl_kinova_init)
        return;

    int kinova_move = 0;
    kinova_move = Kinova_MoveHome();

    fl_kinova_init_position = true;

    emit SignalKinovaPosition(KinovaGetPosition());
}

void CKinova::KinovaResetFactorial(){
    Kinova_RestoreFactoryDefault();
}

bool CKinova::Kinova_Scan_Init(double _rel_pos,double unit_step/*m*/,CartesianPosition _ref_position){

    m_scan_unit_step = _rel_pos > 0 ? (1) * unit_step : (-1) * unit_step;

    m_scan_next_z = 0;
    m_scan_current_z = 0;

    m_scan_final_z = _ref_position.Coordinates.Z + _rel_pos;
    m_scan_current_step = 0;

    m_cartisian_ref_position.Coordinates.X = _ref_position.Coordinates.X;
    m_cartisian_ref_position.Coordinates.Y = _ref_position.Coordinates.Y;
    m_cartisian_ref_position.Coordinates.Z = _ref_position.Coordinates.Z;

    m_cartisian_ref_position.Coordinates.ThetaX = _ref_position.Coordinates.ThetaX;
    m_cartisian_ref_position.Coordinates.ThetaY = _ref_position.Coordinates.ThetaY;
    m_cartisian_ref_position.Coordinates.ThetaZ = _ref_position.Coordinates.ThetaZ;

    fl_kinova_manipulation = true;

    return true;
}

CartesianPosition CKinova::KinovaGetPosition(){

    CartesianPosition position;
    Kinova_GetCartesianPosition(position);

    return position;
}

CartesianPosition CKinova::KinovaGetCartesianForce(){
    CartesianPosition force;
    Kinova_GetCartesianForce(force);

    return force;
}

void CKinova::KinovaSetCartesianForceThread(){

    if(!fl_kinova_get_force_feedback){
        this->start();
    }

    else
        fl_kinova_get_force_feedback = false;
}

CartesianPosition CKinova::KinovaGetCartesianForceThread(){

    CartesianPosition kinova_force;
    Kinova_GetCartesianForce(kinova_force);

    emit SignalKinovaForce(kinova_force);

    msleep(30);
}

bool CKinova::KinovaMoveUnitStep(double _x, double _y, double _z, double _th_x, double _th_y, double _th_z){

    if(!(fl_kinova_init && fl_kinova_init_position))
        return false;

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    pointToSend.Position.Type = CARTESIAN_VELOCITY;

    pointToSend.Position.CartesianPosition.X = _x;
    pointToSend.Position.CartesianPosition.Y = _y;
    pointToSend.Position.CartesianPosition.Z = _z;
    pointToSend.Position.CartesianPosition.ThetaX = _th_x;
    pointToSend.Position.CartesianPosition.ThetaY = _th_y;
    pointToSend.Position.CartesianPosition.ThetaZ = _th_z;

    for(int i = 0; i<=STEP_NUM; i++)
    {
        KinovaUnitStepMoving(pointToSend);
        usleep(SLEEP_TIME);
    }
    emit SignalKinovaPosition(KinovaGetPosition());
    sleep(0.5);

    return true;
}

bool CKinova::KinovaMoveUnitStepUp(){

    if(!(fl_kinova_init && fl_kinova_init_position))
        return false;

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    pointToSend.Position.Type = CARTESIAN_VELOCITY;

    pointToSend.Position.CartesianPosition.X = 0;
    pointToSend.Position.CartesianPosition.Y = 0;
    pointToSend.Position.CartesianPosition.Z = VEL;
    pointToSend.Position.CartesianPosition.ThetaX = 0;
    pointToSend.Position.CartesianPosition.ThetaY = 0;
    pointToSend.Position.CartesianPosition.ThetaZ = 0;

    for(int i = 0; i<=STEP_NUM; i++)
    {
        KinovaUnitStepMoving(pointToSend);
        usleep(SLEEP_TIME);
    }
    emit SignalKinovaPosition(KinovaGetPosition());
    sleep(0.5);

    return true;
}

bool CKinova::KinovaMoveUnitStepDw(){

    if(!(fl_kinova_init && fl_kinova_init_position))
        return false;

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    pointToSend.Position.Type = CARTESIAN_VELOCITY;

    pointToSend.Position.CartesianPosition.X = 0;
    pointToSend.Position.CartesianPosition.Y = 0;
    pointToSend.Position.CartesianPosition.Z = -VEL;
    pointToSend.Position.CartesianPosition.ThetaX = 0;
    pointToSend.Position.CartesianPosition.ThetaY = 0;
    pointToSend.Position.CartesianPosition.ThetaZ = 0;

    for(int i = 0; i<=STEP_NUM; i++)
    {
        KinovaUnitStepMoving(pointToSend);
        usleep(SLEEP_TIME);
    }
    emit SignalKinovaPosition(KinovaGetPosition());
    sleep(0.5);

    return true;
}

bool CKinova::KinovaMoveUnitStepRi(){

    if(!(fl_kinova_init && fl_kinova_init_position))
        return false;

    CartesianPosition position_z_pose;
    Kinova_GetCartesianPosition(position_z_pose);

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    pointToSend.Position.Type = CARTESIAN_VELOCITY;

    pointToSend.Position.CartesianPosition.X = 0;
    pointToSend.Position.CartesianPosition.Y = -VEL;
    pointToSend.Position.CartesianPosition.Z = 0;
    pointToSend.Position.CartesianPosition.ThetaX = 0;
    pointToSend.Position.CartesianPosition.ThetaY = 0;
    pointToSend.Position.CartesianPosition.ThetaZ = 0;

    for(int i = 0; i<=STEP_NUM; i++)
    {
        KinovaUnitStepMoving(pointToSend);
        usleep(SLEEP_TIME);
    }
    emit SignalKinovaPosition(KinovaGetPosition());
    msleep(300);


    CartesianPosition position_rpy_pose;
    Kinova_GetCartesianPosition(position_rpy_pose);

    position_rpy_pose.Coordinates.X = position_z_pose.Coordinates.X;
    position_rpy_pose.Coordinates.Z = position_z_pose.Coordinates.Z;

    position_rpy_pose.Coordinates.ThetaX = ALIGN_TO_PANEL_YAW_VALUE;
    position_rpy_pose.Coordinates.ThetaY = ALIGN_TO_PANEL_PITCH_VALUE;
    position_rpy_pose.Coordinates.ThetaZ = ALIGN_TO_PANEL_ROLL_VALUE;

    KinovaDoManipulate(position_rpy_pose, 3, 100);

    return true;
}

bool CKinova::KinovaMoveUnitStepLe(){

    if(!(fl_kinova_init && fl_kinova_init_position))
        return false;

    CartesianPosition position_z_pose;
    Kinova_GetCartesianPosition(position_z_pose);

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    pointToSend.Position.Type = CARTESIAN_VELOCITY;

    pointToSend.Position.CartesianPosition.X = 0;
    pointToSend.Position.CartesianPosition.Y = VEL;
    pointToSend.Position.CartesianPosition.Z = 0;
    pointToSend.Position.CartesianPosition.ThetaX = 0;
    pointToSend.Position.CartesianPosition.ThetaY = 0;
    pointToSend.Position.CartesianPosition.ThetaZ = 0;

    for(int i = 0; i<=STEP_NUM; i++)
    {
        KinovaUnitStepMoving(pointToSend);
        usleep(SLEEP_TIME);
    }

    emit SignalKinovaPosition(KinovaGetPosition());
    msleep(300);

    CartesianPosition position_rpy_pose;
    Kinova_GetCartesianPosition(position_rpy_pose);

    position_rpy_pose.Coordinates.X = position_z_pose.Coordinates.X;
    position_rpy_pose.Coordinates.Z = position_z_pose.Coordinates.Z;

    position_rpy_pose.Coordinates.ThetaX = ALIGN_TO_PANEL_YAW_VALUE;
    position_rpy_pose.Coordinates.ThetaY = ALIGN_TO_PANEL_PITCH_VALUE;
    position_rpy_pose.Coordinates.ThetaZ = ALIGN_TO_PANEL_ROLL_VALUE;

    KinovaDoManipulate(position_rpy_pose, 3, 100);

    return true;
}

bool CKinova::KinovaMoveUnitStepFw(){

    if(!(fl_kinova_init && fl_kinova_init_position))
        return false;

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    pointToSend.Position.Type = CARTESIAN_VELOCITY;

    pointToSend.Position.CartesianPosition.X = VEL;
    pointToSend.Position.CartesianPosition.Y = 0;
    pointToSend.Position.CartesianPosition.Z = 0;
    pointToSend.Position.CartesianPosition.ThetaX = 0;
    pointToSend.Position.CartesianPosition.ThetaY = 0;
    pointToSend.Position.CartesianPosition.ThetaZ = 0;

    for(int i = 0; i<=STEP_NUM; i++)
    {
        KinovaUnitStepMoving(pointToSend);
        usleep(SLEEP_TIME);
    }
    emit SignalKinovaPosition(KinovaGetPosition());
    sleep(0.5);

    return true;
}

bool CKinova::KinovaMoveUnitStepBw(){

    if(!(fl_kinova_init && fl_kinova_init_position))
        return false;

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    pointToSend.Position.Type = CARTESIAN_VELOCITY;

    pointToSend.Position.CartesianPosition.X = -VEL;
    pointToSend.Position.CartesianPosition.Y = 0;
    pointToSend.Position.CartesianPosition.Z = 0;
    pointToSend.Position.CartesianPosition.ThetaX = 0;
    pointToSend.Position.CartesianPosition.ThetaY = 0;
    pointToSend.Position.CartesianPosition.ThetaZ = 0;

    for(int i = 0; i<=STEP_NUM; i++)
    {
        KinovaUnitStepMoving(pointToSend);
        usleep(SLEEP_TIME);
    }
    emit SignalKinovaPosition(KinovaGetPosition());
    sleep(0.5);


    return true;
}

void CKinova::SetKinovaRotateValve(bool _using_current_coord, bool _init_angle, double _x, double _y, double _z){

    if(_using_current_coord){
        fl_using_current_coor = true;
    }
    else{
        x_coord = _x;
        y_coord = _y;
        z_coord = _z;

        fl_using_current_coor = false;
    }

    if(_init_angle){
        initialAngle = 0;
    }
}

bool CKinova::KinovaRotateValveMotion(bool _using_constant_c_point,CartesianPosition _center_point, int _radius, int _theta, double _unit_angle/*deg*/){

    CartesianPosition current_position;
    double _center_x = 0;
    double _center_y = 0;
    double _center_z = 0;

    Kinova_GetCartesianPosition(current_position);

    if(_using_constant_c_point){
        if(x_coord == 0)
            x_coord = current_position.Coordinates.X;
        if(y_coord == 0)
            y_coord = current_position.Coordinates.Y - 0.01*_radius*sin(initialAngle*3.141592/180);
        if(z_coord == 0)
            z_coord = current_position.Coordinates.Z - 0.01*_radius*cos(initialAngle*3.141592/180);
    }
    else{

    }

    return true;
}

bool CKinova::KinovaRotateValveMotion(VALVE_ROTATE_DIR _dir, double _radius, int _theta){
    //_radius [cm], _theta [deg]

    CartesianPosition position;
    CartesianPosition current_position;
    TrajectoryPoint desired_point;

    desired_point.InitStruct();

    Kinova_GetCartesianPosition(current_position);

    std::cout << "Check Rotate Valve Motion -2 s" << std::endl;

    if(fl_using_current_coor){
//        x_coord = current_position.Coordinates.X;
//        y_coord = current_position.Coordinates.Y + 0.01*_radius*sin(initialAngle*3.141592/180);
//        z_coord = current_position.Coordinates.Z - 0.01*_radius*cos(initialAngle*3.141592/180);
    }
    else{
        if(x_coord == 0)
            x_coord = current_position.Coordinates.X;
        if(y_coord == 0)
            y_coord = current_position.Coordinates.Y + 0.01*_radius*sin(initialAngle*3.141592/180);
        if(z_coord == 0)
            z_coord = current_position.Coordinates.Z - 0.01*_radius*cos(initialAngle*3.141592/180);
    }

    theta = 0.0;

    double unit_deg = 5;
    for(double i = 1; i <= _theta/unit_deg; i++){

        std::cout << "Check Rotate Valve Motion -3 s" << std::endl;
        if(_dir == CW){
            theta -= unit_deg*3.14159265359/180.0;
        }
        else if(_dir == CCW){
            theta += unit_deg*3.14159265359/180.0;
        }

        position.Coordinates.X = x_coord; //0.38487;
        position.Coordinates.Y = y_coord + _radius * 0.01 * sin(initialAngle * 3.14159265359 / 180.0 + theta); //0.22056 - _param2*0.01*sin(theta);
        position.Coordinates.Z = z_coord + _radius * 0.01 * cos(initialAngle * 3.14159265359 / 180.0 + theta); //0.28821 + _param2*0.01*cos(theta);

        position.Coordinates.ThetaZ = ALIGN_TO_PANEL_ROLL_VALUE - initialAngle * 3.14159265359/180.0 - theta;
        position.Coordinates.ThetaY = ALIGN_TO_PANEL_PITCH_VALUE + 0.04;
        position.Coordinates.ThetaX = ALIGN_TO_PANEL_YAW_VALUE;

        KinovaDoManipulate(position, 4);

        _radius -= 0.05;

        if(fabs(_theta) < 120){
            msleep(600);
        }
        else {
            msleep(50);
        }
        std::cout << "Check Rotate Valve Motion -3 e" << std::endl;
    }

    if(_dir == CW){
        initialAngle -= _theta;
    }
    else if(_dir == CCW){
        initialAngle += _theta;
    }

    std::cout << "Check Rotate Valve Motion -2 e" << std::endl;
    return true;
}

void CKinova::Kinova_Scan_End(){

    m_scan_final_z = 0;
    m_scan_unit_step = 0;
    m_scan_current_step = 0;

    m_cartisian_ref_position.Coordinates.X = 0;
    m_cartisian_ref_position.Coordinates.Y = 0;
    m_cartisian_ref_position.Coordinates.Z = 0;

    m_cartisian_ref_position.Coordinates.ThetaX = 0;
    m_cartisian_ref_position.Coordinates.ThetaY = 0;
    m_cartisian_ref_position.Coordinates.ThetaZ = 0;
}

bool CKinova::Kinova_Scan_Moving(){

    if(!fl_kinova_init)
        return false;

    TrajectoryPoint nex_step_position;

    nex_step_position.InitStruct();
    nex_step_position.Position.Type = CARTESIAN_POSITION;

    nex_step_position.Position.CartesianPosition.X = m_cartisian_ref_position.Coordinates.X;
    nex_step_position.Position.CartesianPosition.Y = m_cartisian_ref_position.Coordinates.Y;
    //Next Z Position
    nex_step_position.Position.CartesianPosition.Z = m_cartisian_ref_position.Coordinates.Z + m_scan_unit_step * (m_scan_current_step);

    nex_step_position.Position.CartesianPosition.ThetaX = m_cartisian_ref_position.Coordinates.ThetaX;
    nex_step_position.Position.CartesianPosition.ThetaY = m_cartisian_ref_position.Coordinates.ThetaY;
    nex_step_position.Position.CartesianPosition.ThetaZ = m_cartisian_ref_position.Coordinates.ThetaZ;

//    Kinova_SendAdvanceTrajectory(nex_step_position);

    Kinova_Scan_Moving(m_scan_unit_step * (m_scan_current_step), m_cartisian_ref_position);

    CartesianPosition position;
    Kinova_GetCartesianPosition(position);

    if(m_scan_unit_step > 0){
        if(position.Coordinates.Z > m_cartisian_ref_position.Coordinates.Z)//Approach Reference Position
            fl_kinova_manipulation = false;
    }
    else{
        if(position.Coordinates.Z < m_cartisian_ref_position.Coordinates.Z)//Approach Reference Position
            fl_kinova_manipulation = false;
    }

    msleep(100);

    m_scan_current_step++;

    emit Get_Kinova_Position(position);
    return true;
}

bool CKinova::Kinova_Scan_Moving(double _rel_pos,CartesianPosition _ref_position){//Up, Down Moving From Current Position to Relative Position

//    double unit_step = 0.01;//m
    CartesianPosition position;
    TrajectoryPoint desired_position;

    desired_position.InitStruct();
    desired_position.Position.Type = CARTESIAN_POSITION;

    Kinova_GetCartesianPosition(position);

    desired_position.Position.CartesianPosition.X = _ref_position.Coordinates.X;
    desired_position.Position.CartesianPosition.Y = _ref_position.Coordinates.Y;
    //Final Z Position
    desired_position.Position.CartesianPosition.Z = _ref_position.Coordinates.Z + _rel_pos;

    desired_position.Position.CartesianPosition.ThetaX = _ref_position.Coordinates.ThetaX;
    desired_position.Position.CartesianPosition.ThetaY = _ref_position.Coordinates.ThetaY;
    desired_position.Position.CartesianPosition.ThetaZ = _ref_position.Coordinates.ThetaZ;

    if(!fl_kinova_init)
        return false;

    double error0;
    std::cout << "Start Scan Motion!" << std::endl;

    Kinova_EraseAllTrajectories();

    Kinova_GetCartesianPosition(position);

    error0 =  pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
            + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
            + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

    error0 = sqrt(error0);

    double error = error0;
    double error_old = error0;
    double thresh = 0.003;
    double thresh2 = 1e-4;
    int count = 0;

    while(error>thresh && count < 15 )
    {
        if(fabs(error-error_old)<thresh2 && error0>=error)
        {
            count++;
        }
        else
        {
            count = 0;
        }

        error_old = error;

        Kinova_GetCartesianPosition(position);
        error =   pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);
        error = sqrt(error);

        Kinova_SendAdvanceTrajectory(desired_position);
    }

//    Kinova_GetCartesianPosition(position);
    msleep(100);

    //emit Get_Kinova_Position(position);
    return true;
}

double CKinova::Kinova_Get_Rel_Pos_Z(CartesianPosition _ref_position){//Up, Down Moving From Current Position to Relative Position

    CartesianPosition position;

    Kinova_GetCartesianPosition(position);

    return (position.Coordinates.Z - _ref_position.Coordinates.Z);
}

void CKinova::Kinova_Do_Scan(double _kinova_rel_pos, CartesianPosition _desired_position){

    if(fl_kinova_manipulation){
        std::cout << "Kinova Busy!!" << std::endl;
        return;
    }
    m_kinova_rel_pos = _kinova_rel_pos;
    m_kinova_disired_position = _desired_position;

    fl_kinova_manipulation = true;

    this->start();
}

void CKinova::Kinova_Do_Wrench_Scan(double _kinova_rel_pos){
    if(fl_kinova_manipulation){
        std::cout << "Kinova Busy!!" << std::endl;
        return;
    }
    m_kinova_rel_pos = _kinova_rel_pos;

    CartesianPosition position;
    Kinova_GetCartesianPosition(position);

    m_kinova_disired_position = position;

    fl_kinova_manipulation = true;

    this->start();
}

//--------------------------------------------------
//
// Complex Motion
//
//--------------------------------------------------

void CKinova::KinovaAlignToPanel(){

//    KinovaInitMotion();
//    msleep(500);

    Kinova_EraseAllTrajectories();

    TrajectoryPoint desired_point;
    desired_point.InitStruct();

    desired_point.Position.CartesianPosition.X = 0.1801; //0.38487;
    desired_point.Position.CartesianPosition.Y = -0.2961; //0.22056;
    desired_point.Position.CartesianPosition.Z = 0.3877; //0.28821 + _param2*0.01;
    desired_point.Position.CartesianPosition.ThetaZ = ALIGN_TO_PANEL_ROLL_VALUE;//-1.1;//0.2535;
    desired_point.Position.CartesianPosition.ThetaY = ALIGN_TO_PANEL_PITCH_VALUE;//+m_base_frame_rot*KINOVA_PI/180;
    desired_point.Position.CartesianPosition.ThetaX = ALIGN_TO_PANEL_YAW_VALUE; //1.39 0.2535

//    desired_point.Position.CartesianPosition.X = 0.1797; //0.38487;
//    desired_point.Position.CartesianPosition.Y = -0.2992; //0.22056;
//    desired_point.Position.CartesianPosition.Z = 0.3900; //0.28821 + _param2*0.01;
//    desired_point.Position.CartesianPosition.ThetaZ = -0.0656;//-1.1;//0.2535;
//    desired_point.Position.CartesianPosition.ThetaY = 1.57;//+m_base_frame_rot*KINOVA_PI/180;
//    desired_point.Position.CartesianPosition.ThetaX = 1.57; //1.39 0.2535

    Kinova_SendBasicTrajectory(desired_point);

    msleep(1000);

    initialAngle = 0;

    emit SignalKinovaPosition(KinovaGetPosition());
}

void CKinova::KinovaRotateBase(double _rot_deg){

    m_base_frame_rot = _rot_deg;

    CartesianPosition current_pose;
    Kinova_GetCartesianPosition(current_pose);

    KinovaDoManipulate(current_pose,2,10);
}

void CKinova::Kinova_GoTo_Wrench(){

    Kinova_EraseAllTrajectories();
    TrajectoryPoint desired_point;
    desired_point.InitStruct();
    desired_point.Position.Type = CARTESIAN_POSITION;

    desired_point.Position.CartesianPosition.X = 0.40; //0.38487;
    desired_point.Position.CartesianPosition.Y = -0.10; //0.22056;
    desired_point.Position.CartesianPosition.Z = 0.27; //0.28821 + _param2*0.01;
    desired_point.Position.CartesianPosition.ThetaZ = -0.5635;
    desired_point.Position.CartesianPosition.ThetaY = 1.57;
    desired_point.Position.CartesianPosition.ThetaX = 1.57;
    Kinova_Do_Manipulate(desired_point, 2);
//    Kinova_SendBasicTrajectory(desired_point);
//    usleep(3000);
    Kinova_EraseAllTrajectories();

    desired_point.Position.CartesianPosition.X = 0.25; //0.38487;
    desired_point.Position.CartesianPosition.Y = -0.30; //0.22056;
    desired_point.Position.CartesianPosition.Z = 0.33; //0.28821 + _param2*0.01;
    desired_point.Position.CartesianPosition.ThetaZ = -0.5635;
    desired_point.Position.CartesianPosition.ThetaY = 1.57;
    desired_point.Position.CartesianPosition.ThetaX = 1.57;
    Kinova_Do_Manipulate(desired_point, 2);
//    Kinova_SendBasicTrajectory(desired_point);
}

void CKinova::Kinova_GoTo_Valve(){

    Kinova_EraseAllTrajectories();

    TrajectoryPoint desired_point;
    desired_point.InitStruct();

    desired_point.Position.CartesianPosition.X = 0.40; //0.38487;
    desired_point.Position.CartesianPosition.Y = 0.16; //0.22056;
    desired_point.Position.CartesianPosition.Z = 0.22; //0.28821 + _param2*0.01;
    desired_point.Position.CartesianPosition.ThetaZ = -0.5;
    desired_point.Position.CartesianPosition.ThetaY = 1.57;
    desired_point.Position.CartesianPosition.ThetaX = 1.57;

    Kinova_SendBasicTrajectory(desired_point);

}
void CKinova::Kinova_Ready_To_Grasp(){
    Kinova_EraseAllTrajectories();
    TrajectoryPoint desired_point;
    desired_point.InitStruct();
    desired_point.Position.Type = CARTESIAN_POSITION;

    desired_point.Position.CartesianPosition.X = 0.565; //0.38487;
    desired_point.Position.CartesianPosition.Y = -0.217; //0.22056;
    desired_point.Position.CartesianPosition.Z = 0.41; //0.28821 + _param2*0.01;
    desired_point.Position.CartesianPosition.ThetaZ = -0.5635;
    desired_point.Position.CartesianPosition.ThetaY = 1.57;
    desired_point.Position.CartesianPosition.ThetaX = 1.57;
    Kinova_Do_Manipulate(desired_point, 2);

}

void CKinova::Kinova_Ready_To_Rotate(){
    Kinova_EraseAllTrajectories();
    TrajectoryPoint desired_point;
    desired_point.InitStruct();
    desired_point.Position.Type = CARTESIAN_POSITION;

    desired_point.Position.CartesianPosition.X = 0.48; //0.38487;
    desired_point.Position.CartesianPosition.Y = -0.207; //0.22056;
    desired_point.Position.CartesianPosition.Z = 0.41; //0.28821 + _param2*0.01;
    desired_point.Position.CartesianPosition.ThetaZ = -0.5635;
    desired_point.Position.CartesianPosition.ThetaY = 1.57;
    desired_point.Position.CartesianPosition.ThetaX = 1.57;
    Kinova_Do_Manipulate(desired_point, 2);

    Kinova_EraseAllTrajectories();

    desired_point.Position.CartesianPosition.X = 0.4893; //0.452;
    desired_point.Position.CartesianPosition.Y = 0.17058; //0.1655;
    desired_point.Position.CartesianPosition.Z = 0.44; //0.28821 + _param2*0.01;
    desired_point.Position.CartesianPosition.ThetaZ = -0.22; //r
    desired_point.Position.CartesianPosition.ThetaY = 1.57; //p
    desired_point.Position.CartesianPosition.ThetaX = 1.57; //y
    Kinova_Do_Manipulate(desired_point, 2);

}

void CKinova::Kinova_Wrench_Injection(){
    Kinova_EraseAllTrajectories();
    TrajectoryPoint desired_point;
    desired_point.InitStruct();
    desired_point.Position.Type = CARTESIAN_POSITION;

    desired_point.Position.CartesianPosition.X = 0.4743; //0.43;
    desired_point.Position.CartesianPosition.Y = 0.17158; //0.19536;
    desired_point.Position.CartesianPosition.Z = 0.385; //0.395;
    desired_point.Position.CartesianPosition.ThetaZ = -0.2;
    desired_point.Position.CartesianPosition.ThetaY = 1.57;
    desired_point.Position.CartesianPosition.ThetaX = 1.57;
    Kinova_Do_Manipulate(desired_point, 2);

}


//--------------------------------------------------

bool CKinova::IsScanning(){

    return fl_kinova_manipulation;
}

bool CKinova::KinovaDoManipulate(CartesianPosition _desired_position,int _mode, double _force_threshold){

    if(!fl_kinova_init)
        return false;

    Kinova_EraseAllTrajectories();

    CartesianPosition position;

    TrajectoryPoint desired_position;

    desired_position.InitStruct();
    desired_position.Position.Type = CARTESIAN_POSITION;

    desired_position.Position.CartesianPosition.X = _desired_position.Coordinates.X*cos(m_base_frame_rot*KINOVA_PI/180) - _desired_position.Coordinates.Y*sin(m_base_frame_rot*KINOVA_PI/180);
    desired_position.Position.CartesianPosition.Y = _desired_position.Coordinates.Y*cos(m_base_frame_rot*KINOVA_PI/180) + _desired_position.Coordinates.X*sin(m_base_frame_rot*KINOVA_PI/180);
    desired_position.Position.CartesianPosition.Z = _desired_position.Coordinates.Z;

    desired_position.Position.CartesianPosition.ThetaX = _desired_position.Coordinates.ThetaX;//+m_base_frame_rot*KINOVA_PI/180;
    desired_position.Position.CartesianPosition.ThetaY = _desired_position.Coordinates.ThetaY+m_base_frame_rot*KINOVA_PI/180;
    desired_position.Position.CartesianPosition.ThetaZ = _desired_position.Coordinates.ThetaZ;//+m_base_frame_rot*KINOVA_PI/180;


    if(_mode == 1){//Joystick Mode

        Kinova_SendBasicTrajectory(desired_position);

        Kinova_GetCartesianPosition(position);

        Kinova_EraseAllTrajectories();
    }

    else if(_mode == 2){ // Mode 2 : Trajectory Mode
        double error0;

        Kinova_GetCartesianPosition(position);

        error0 =  pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

        error0 = sqrt(error0);

        int count = 0;

        double error = error0;
        double error_old = error0;
        double thresh = 0.003;
        double thresh2 = 1e-3;

        while(error>thresh && count < 5)
        {
//            if(_force_threshold != 0){
//                if(KinovaGetCartesianForce().Coordinates.X > _force_threshold)
//                    break;
//                if(KinovaGetCartesianForce().Coordinates.Y > _force_threshold)
//                    break;
//                if(KinovaGetCartesianForce().Coordinates.Z > _force_threshold)
//                    break;
//            }

            if(fabs(error-error_old)<thresh2 && error0>error)
            {
                count++;
            }
            else
            {
                count = 0;
            }

            error_old = error;

            Kinova_GetCartesianPosition(position);
            error =   pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                    + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                    + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

            error = sqrt(error);

//            std::cout << "X: "<< desired_position.Position.CartesianPosition.X <<std::endl;
//            std::cout << "Y: "<< desired_position.Position.CartesianPosition.Y <<std::endl;
//            std::cout << "Z: "<< desired_position.Position.CartesianPosition.Z <<std::endl;

//            std::cout << "Ro: "<< desired_position.Position.CartesianPosition.ThetaZ <<std::endl;
//            std::cout << "Pi: "<< desired_position.Position.CartesianPosition.ThetaY <<std::endl;
//            std::cout << "Ya: "<< desired_position.Position.CartesianPosition.ThetaX <<std::endl;

            Kinova_SendAdvanceTrajectory(desired_position);
            msleep(10);
        }
    }
    else if(_mode == 3){ // Mode 3 : Roll
        double error0;
        Kinova_SendAdvanceTrajectory(desired_position);
        msleep(100);

//        Kinova_GetCartesianPosition(position);

//        error0 =  pow((desired_position.Position.CartesianPosition.ThetaX - position.Coordinates.ThetaX),2)
//                + pow((desired_position.Position.CartesianPosition.ThetaY - position.Coordinates.ThetaY),2)
//                + pow((desired_position.Position.CartesianPosition.ThetaZ - position.Coordinates.ThetaZ),2);

//        error0 = sqrt(error0);

//        int count = 0;

//        double error = error0;
//        double error_old = error0;
//        double thresh = KINOVA_PI / 180;
//        double thresh2 = (KINOVA_PI / 180);

//        while((error > thresh) && (count < 5)){

//            if((fabs(error - error_old) < thresh2) && (error0 >= error))
//            {
//                count++;
//            }
//            else
//            {
//                count = 0;
//            }

//            error_old = error;

//            Kinova_GetCartesianPosition(position);
//            error =   pow((desired_position.Position.CartesianPosition.ThetaX - position.Coordinates.ThetaX),2)
//                    + pow((desired_position.Position.CartesianPosition.ThetaY - position.Coordinates.ThetaY),2)
//                    + pow((desired_position.Position.CartesianPosition.ThetaZ - position.Coordinates.ThetaZ),2);

//            error = sqrt(error);

//            Kinova_SendAdvanceTrajectory(desired_position);
//        }
    }
    else if(_mode == 4){ // Mode 2 : Trajectory Mode
        double error0;

        std::cout << "Check Rotate Valve Motion -4 s" << std::endl;
        Kinova_GetCartesianPosition(position);

        error0 =  pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

        error0 = sqrt(error0);

        int count = 0;

        double error = error0;
        double error_old = error0;
        double thresh = 0.01;
        double thresh2 = 1e-1;

        std::cout << "Check Rotate Valve Motion -4 e" << std::endl;

        while(error>thresh && count < 5)
        {

            std::cout << "Check Rotate Valve Motion -5 s" << std::endl;
            if(fabs(error-error_old)<thresh2 && error0>error)
            {
                count++;
            }
            else
            {
                count = 0;
            }

            error_old = error;

            std::cout << "Check Rotate Valve Motion -6 s" << std::endl;
            Kinova_GetCartesianPosition(position);
            std::cout << "Check Rotate Valve Motion -6 e" << std::endl;
            error =   pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                    + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                    + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

            error = sqrt(error);

            Kinova_SendAdvanceTrajectory(desired_position);
            msleep(10);
            std::cout << "Check Rotate Valve Motion -5 e" << std::endl;
        }
    }

    else if(_mode == 5){ // Mode 5: Only X-Y-Z Position
        double error0;

        Kinova_GetCartesianPosition(position);

        error0 =  pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

        error0 = sqrt(error0);

        int count = 0;

        double error = error0;
        double error_old = error0;
        double thresh = 0.01;
        double thresh2 = 1e-1;

        while(error>thresh && count < 5)
        {

            if(fabs(error-error_old)<thresh2 && error0>error)
            {
                count++;
            }
            else
            {
                count = 0;
            }

            error_old = error;

            Kinova_GetCartesianPosition(position);
            error =   pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                    + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                    + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

            error = sqrt(error);

            desired_position.Position.CartesianPosition.ThetaX = position.Coordinates.ThetaX;
            desired_position.Position.CartesianPosition.ThetaY = position.Coordinates.ThetaY;
            desired_position.Position.CartesianPosition.ThetaZ = position.Coordinates.ThetaZ;

            Kinova_SendAdvanceTrajectory(desired_position);

            msleep(10);
        }
    }

    else if(_mode == 6){ // Mode 5: Only Roll-Pitch-Yaw Position
        double error0;

        Kinova_GetCartesianPosition(position);

        error0 =  pow((desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                + pow((desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                + pow((desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

        error0 = sqrt(error0);

        int count = 0;

        double error = error0;
        double error_old = error0;
        double thresh = 0.01;
        double thresh2 = 1e-1;

        while(error>thresh)
        {

            if(fabs(error-error_old)<thresh2 && error0>error)
            {
                count++;
            }
            else
            {
                count = 0;
            }

            if(count > 5)
                return false;

            error_old = error;

            Kinova_GetCartesianPosition(position);

            error =   pow((desired_position.Position.CartesianPosition.ThetaX - position.Coordinates.ThetaX),2)
                    + pow((desired_position.Position.CartesianPosition.ThetaY - position.Coordinates.ThetaY),2)
                    + pow((desired_position.Position.CartesianPosition.ThetaZ - position.Coordinates.ThetaZ),2);

            error = sqrt(error);

            desired_position.Position.CartesianPosition.X = position.Coordinates.X;
            desired_position.Position.CartesianPosition.Y = position.Coordinates.Y;
            desired_position.Position.CartesianPosition.Z = position.Coordinates.Z;

            Kinova_SendAdvanceTrajectory(desired_position);

            msleep(10);
        }
    }

    emit SignalKinovaPosition(KinovaGetPosition());
    return true;
}

bool CKinova::KinovaDoManipulate(CartesianPosition _desired_position, double _force_threshold){

    if(!fl_kinova_init)
        return false;

    Kinova_EraseAllTrajectories();

    CartesianPosition position;

    TrajectoryPoint desired_position;

    desired_position.InitStruct();
    desired_position.Position.Type = CARTESIAN_POSITION;

    desired_position.Position.CartesianPosition.X = _desired_position.Coordinates.X;
    desired_position.Position.CartesianPosition.Y = _desired_position.Coordinates.Y;
    desired_position.Position.CartesianPosition.Z = _desired_position.Coordinates.Z;

    desired_position.Position.CartesianPosition.ThetaX = _desired_position.Coordinates.ThetaX;
    desired_position.Position.CartesianPosition.ThetaY = _desired_position.Coordinates.ThetaY;
    desired_position.Position.CartesianPosition.ThetaZ = _desired_position.Coordinates.ThetaZ;

    double error_th = 0.03;
    double error_th_th = 0.02;

    double error_x;
    double error_y;
    double error_z;

    double error_th_x;
    double error_th_y;
    double error_th_z;

    do{
        Kinova_GetCartesianPosition(position);

        error_x = desired_position.Position.CartesianPosition.X - position.Coordinates.X;
        error_y = desired_position.Position.CartesianPosition.Y - position.Coordinates.Y;
        error_z = desired_position.Position.CartesianPosition.Z - position.Coordinates.Z;

        error_th_x = desired_position.Position.CartesianPosition.ThetaX - position.Coordinates.ThetaX;
        error_th_y = desired_position.Position.CartesianPosition.ThetaY - position.Coordinates.ThetaY;
        error_th_z = desired_position.Position.CartesianPosition.ThetaZ - position.Coordinates.ThetaZ;

        if(fabs(error_x) > error_th){
             error_x = error_x > 0 ? VEL : (-1)*VEL;
        }
        else
            error_x = 0;
        if(fabs(error_y) > error_th){
             error_y = error_y > 0 ? VEL : (-1)*VEL;
        }
        else
            error_y = 0;
        if(fabs(error_z) > error_th){
             error_z = error_z > 0 ? VEL : (-1)*VEL;
        }
        else
            error_z = 0;

        if(fabs(error_th_x) > error_th_th){
             error_th_x = error_th_x > 0 ? (-1)*ROT : ROT;
        }
        else
            error_th_x = 0;
        if(fabs(error_th_y) > error_th_th){
             error_th_y = error_th_y > 0 ? (-1)*ROT : ROT;
        }
        else
            error_th_y = 0;
        if(fabs(error_th_z) > error_th_th){
             error_th_z = error_th_z > 0 ? (-1)*ROT : ROT;
        }
        else
            error_th_z = 0;

        KinovaMoveUnitStep(error_x, error_y, error_z, error_th_x, error_th_y, error_th_z);

        if(_force_threshold != 0){
            if(KinovaGetCartesianForce().Coordinates.X > _force_threshold)
                break;
            if(KinovaGetCartesianForce().Coordinates.Y > _force_threshold)
                break;
            if(KinovaGetCartesianForce().Coordinates.Z > _force_threshold)
                break;
        }

        msleep(10);
    }while((error_x != 0) || (error_y != 0) || (error_z != 0) ||
           (error_th_x != 0) || (error_th_y != 0) || (error_th_z != 0));

    emit SignalKinovaPosition(KinovaGetPosition());

    return true;
}

bool CKinova::Kinova_Do_Manipulate(TrajectoryPoint _desired_position,int _mode){

    if(!fl_kinova_init)
        return false;

    CartesianPosition position;

    if(_mode == 1){//Joystick Mode

        Kinova_SendBasicTrajectory(_desired_position);

        Kinova_GetCartesianPosition(position);

        emit Get_Kinova_Position(position);

        Kinova_EraseAllTrajectories();

        return true;
    }

    else{ // Mode 2 : Trajectory Mode
        double error0;

        Kinova_GetCartesianPosition(position);

        error0 =  pow((_desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                + pow((_desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                + pow((_desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);

        error0 = sqrt(error0);

        double error = error0;
        double error_old = error0;
        double thresh = 0.003;
        double thresh2 = 1e-6;
        int count = 0;

        std::cout << "X : "<< _desired_position.Position.CartesianPosition.X << std::endl;
        std::cout << "Y : "<< _desired_position.Position.CartesianPosition.Y << std::endl;
        std::cout << "Z : "<< _desired_position.Position.CartesianPosition.Z << std::endl;

        while(error>thresh && count < 5 )
        {
            if(fabs(error-error_old)<thresh2 && error0>error)
            {
                count++;
            }
            else
            {
                count = 0;
            }

            error_old = error;

            Kinova_GetCartesianPosition(position);
            error =   pow((_desired_position.Position.CartesianPosition.X - position.Coordinates.X),2)
                    + pow((_desired_position.Position.CartesianPosition.Y - position.Coordinates.Y),2)
                    + pow((_desired_position.Position.CartesianPosition.Z - position.Coordinates.Z),2);
            error = sqrt(error);

            std::cout << "error : "<< error << std::endl;

            Kinova_SendAdvanceTrajectory(_desired_position);
        }

        emit Get_Kinova_Position(position);

        return true;
    }
}

bool CKinova::Kinova_Do_Manipulate(JoystickCommand _desired_command){

    if(!fl_kinova_init)
        return false;

    Kinova_SendJoystickCommand(_desired_command);
    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CKinova::run(){

    if(fl_kinova_manipulation){
        Kinova_Scan_Moving(m_kinova_rel_pos, m_kinova_disired_position);
        std::cout << "End Scan Motion!" << std::endl;
        fl_kinova_manipulation = false;
    }

    if(fl_kinova_get_force_feedback){
        KinovaGetCartesianForceThread();
    }
}
