#include "EurecarURV_Dlg.h"
#include "ui_EurecarURV_Dlg.h"

EurecarURV_Dlg::EurecarURV_Dlg(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EurecarURV_Dlg)
{
    ui->setupUi(this);


    qRegisterMetaType<QVector<int>>("QVector<int>");
    qRegisterMetaType<QVector<double>>("QVector<double>");
    qRegisterMetaType<GRIPPER_STATUS>("GRIPPER_STATUS");
    qRegisterMetaType<CartesianPosition>("CartesianPosition");
    qRegisterMetaType<LRF_VEHICLE_ANGLE_STRUCT>("LRF_VEHICLE_ANGLE_STRUCT");
    qRegisterMetaType<LRF_VEHICLE_HORIZEN_STRUCT>("LRF_VEHICLE_HORIZEN_STRUCT");
    qRegisterMetaType<LRF_KINOVA_ANGLE_CTRL_STRUCT>("LRF_KINOVA_ANGLE_CTRL_STRUCT");
    qRegisterMetaType<LRF_KINOVA_HORIZEN_CTRL_STRUCT>("LRF_KINOVA_HORIZEN_CTRL_STRUCT");
    qRegisterMetaType<LRF_KINOVA_VERTICAL_CTRL_STRUCT>("LRF_KINOVA_VERTICAL_CTRL_STRUCT");

    //-------------------------------------------------
    // Device Class Initialize
    //-------------------------------------------------
    mpc_imu = new CIMU;
    mpc_gps = new CGPS();
    mpc_drive_lrf = new CLRF;
    mpc_mani__lrf = new CLRF;
    mpc_camera = new CCamera;
    mpc_kinova = new CKinova;
    mpc_vehicle = new CVehicle;
    mpc_gripper = new CGripper;
    mpc_pcl = new CPCL;
    mpc_lms511 = new CLMS511(mpc_pcl);

    mpc_ssd = new CSSD;
    mpc_velodyne = new CVelodyne(mpc_pcl);
    //-------------------------------------------------
    // Mission Class Initialize
    //-------------------------------------------------

    //Driving Class Initialize
    mpc_drivig = new CDriving(mpc_imu, mpc_gps, mpc_drive_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne, mpc_lms511);

    //Manipulation Class Initialize
    mpc_manipulation = new CManipulation(mpc_mani__lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne, mpc_gripper, mpc_ssd);

    //-------------------------------------------------
    // Script Class Initialize
    //-------------------------------------------------
    mpc_script = new CScript(mpc_drivig, mpc_manipulation);

    //-------------------------------------------------
    // SubDialog Initialize
    //-------------------------------------------------

    //Driving Dialog Box Initialize
    mpdlg_driving = new Driving_Dlg(mpc_drivig);

    //Manipulation Dialog Box Initialize
    mpdlg_manipulation = new Manipulation_Dlg(mpc_manipulation);

    //-------------------------------------------------
    // Connection
    //-------------------------------------------------

    //Menu Button Connect
    connect(ui->action_driving_dialog_box,SIGNAL(triggered()),this,SLOT(SlotMenuButtonDriving_Dlg()));
    connect(ui->action_manipulation_dialog_box,SIGNAL(triggered()),this,SLOT(SlotMenuButtonManipulation_Dlg()));
    connect(ui->action_load_scenario_script,SIGNAL(triggered()),this,SLOT(SlotMenuButtonScenarioLoad()));

    //Push Button Connect
    connect(ui->bt_velodyne,SIGNAL(clicked()), this, SLOT(SlotButtonVelodyneSwitch()));
    connect(ui->bt_kinova,SIGNAL(clicked()), this, SLOT(SlotButtonKinovaSwitch()));
    connect(ui->bt_mission_run,SIGNAL(clicked()), this, SLOT(SlotButtonMissionRun()));

    connect(ui->bt_kinova_factory_reset,SIGNAL(clicked()), this, SLOT(SlotButtonKinovaReset()));

    connect(ui->bt_mission_partial_run,SIGNAL(clicked()), this, SLOT(SlotButtonMissionPartialRun()));
    connect(ui->bt_scenario_partial_run,SIGNAL(clicked()), this, SLOT(SlotButtonScenarioPartialRun()));
    connect(ui->bt_script_update,SIGNAL(clicked()), this, SLOT(SlotButtonScenarioLoad()));
    connect(ui->bt_mission_pause,SIGNAL(clicked()), this, SLOT(SlotButtonMissionPause()));
    connect(ui->bt_mission_terminate,SIGNAL(clicked()), this, SLOT(SlotButtonMissionTerminate()));

    //Device Button
    connect(ui->bt_mani_lrf,SIGNAL(clicked()), this, SLOT(SlotButtonLRFManiSwitch()));
    connect(ui->bt_drive_lrf,SIGNAL(clicked()), this, SLOT(SlotButtonLRFDriveSwitch()));
    connect(ui->bt_ugv,SIGNAL(clicked()), this, SLOT(SlotButtonVehicleSwitch()));
    connect(ui->bt_gripper,SIGNAL(clicked()), this, SLOT(SlotButtonGripperSwitch()));
    connect(ui->bt_gps,SIGNAL(clicked()), this, SLOT(SlotButtonGPSSwitch()));
    connect(ui->bt_gps_init_pos,SIGNAL(clicked()), this, SLOT(SlotButtonGPSInitPosSwitch()));
    connect(ui->bt_imu,SIGNAL(clicked()), this, SLOT(SlotButtonIMUSwitch()));
    connect(ui->bt_lms511,SIGNAL(clicked()), this, SLOT(SlotButtonLMS511Switch()));
    connect(ui->bt_rotator,SIGNAL(clicked()), this, SLOT(SlotButtonRotatorSwitch()));

    //Click List View
    connect(ui->lsview_mission_title,SIGNAL(clicked(QModelIndex)), this, SLOT(SlotMissionListUpdate(QModelIndex)));
    //Click Combo Box
    connect(ui->cb_step_title,SIGNAL(activated(int)), this, SLOT(SlotStepListUpdate(int)));

    //View System Message
    connect(mpc_script,SIGNAL(SignalScriptMessage(QString)), this, SLOT(SlotViewSystemMessage(QString)));

    //-------------------------------------------------
    // PCL Init For Velodyne
    //-------------------------------------------------
    mpc_drivig->PCLInit();
    ui->qvtk_velodyne_main_dlg->SetRenderWindow(mpc_drivig->GetPCL()->viewer->getRenderWindow());
    (mpc_drivig->GetPCL())->viewer->setupInteractor(ui->qvtk_velodyne_main_dlg->GetInteractor(),ui->qvtk_velodyne_main_dlg->GetRenderWindow());
    (mpc_drivig->GetPCL())->viewer->setBackgroundColor(0,0,0);
    (mpc_drivig->GetPCL())->viewer->addCoordinateSystem(1.0);
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->cloud, "cloud");
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->lms511_cloud, "lms511_cloud");
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->waypoint_cloud, "waypoint_cloud");
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->panelpoint_cloud, "panelpoint_cloud");
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->lrf_cloud, "lrf_cloud");
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->lrf_waypoint_cloud, "lrf_waypoint_cloud");
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->mission_boundary_cloud,"mission_boundary_cloud");

    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"cloud");
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"lms511_cloud");
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"waypoint_cloud");
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"panelpoint_cloud");
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"lrf_cloud");
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"lrf_waypoint_cloud");
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,20,"mission_boundary_cloud");



    // Velodyne View Connect
    connect(mpc_drivig,SIGNAL(SignalVelodyneParser(bool)),this,SLOT(SlotVeloyneParser(bool)));

    //-------------------------------------------------
    // Edite Init
    //-------------------------------------------------

    ui->ed_script_mission_s->setText("0");
    ui->ed_script_mission_f->setText("0");

    ui->ed_script_step_s->setText("0");
    ui->ed_script_step_s->setText("0");

    //-------------------------------------------------
    // Script Update
    //-------------------------------------------------
    QString file_path = "/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/Script/Scenario.md";

    if(!mpc_script->InterpreteScenarioScriptFile(file_path))
        QMessageBox::information(this, tr("Fail to Open Scenario"), tr("Check the Scenario File"));

    ScriptInfoDisplay();
}

EurecarURV_Dlg::~EurecarURV_Dlg()
{
    delete mpc_gps;
    delete mpc_drive_lrf;
    delete mpc_mani__lrf;
    delete mpc_camera;
    delete mpc_kinova;
    delete mpc_vehicle;
    delete mpc_velodyne;

    delete mpc_ssd;

    delete ui;
}

//-------------------------------------------------
// Button
//-------------------------------------------------

void EurecarURV_Dlg::SlotButtonGripperSwitch(){

    if(!mpc_gripper->IsGripperInit()){
        if(!mpc_gripper->InitGripper()){
            QMessageBox::information(this, tr("Fail to Connect Gripper"), tr("Check Gripper"));
        }
        else{
            ui->bt_gripper->setText("Gripper Off");
        }
    }
    else{
        mpc_gripper->CloseGripper();
        ui->bt_gripper->setText("Gripper On");
    }


}

void EurecarURV_Dlg::SlotButtonIMUSwitch(){
    if(!mpc_imu->IsIMUInit())
    {
        if(!mpc_imu->IMUInit(ui->ed_imu_path->text().toStdString(), ui->ed_init_heading->text().toDouble()))
        {
            QMessageBox::information(this, tr("Fail to Connect IMU"),tr("Check IMU"));
        }
        else
        {
            ui->bt_imu->setText("IMU On");
        }
    }
}

void EurecarURV_Dlg::SlotButtonRotatorSwitch(){

    if(!mpc_gripper->IsRotatorInit()){
        if(!mpc_gripper->InitRotator())
        {
            QMessageBox::information(this, tr("Fail to Connect Dyanmixel Pro"),tr("Check Dyanmixel"));
        }
        else
        {
            ui->bt_rotator->setText("Rotator Off");
        }
    }
    else{
        mpc_gripper->CloseRotator();
        ui->bt_rotator->setText("Rotator On");
    }
}

void EurecarURV_Dlg::SlotButtonLMS511Switch(){
    if(!mpc_lms511->IsLMS511Init())
    {
        if(mpc_lms511->ConnectLMS511())
        {
            cout << "LMS 511 init success!" << endl;
            ui->bt_lms511->setText("LMS511 Off");
        }
        else
        {
            cout << "LMS 511 init failed!" << endl;
        }
    }
    else
    {

    }
}

void EurecarURV_Dlg::SlotButtonVelodyneSwitch(){

    if(!mpc_velodyne->IsVelodyneConneted()){
        if(!mpc_velodyne->SetVelodyneThread(true))
            QMessageBox::information(this, tr("Fail to Connect Velodyne"), tr("Check Velodyne"));
        else
            ui->bt_velodyne->setText("Velodyne Off");
    }
    else{
        mpc_velodyne->SetVelodyneThread(false);
        ui->bt_velodyne->setText("Velodyne On");
    }
}

void EurecarURV_Dlg::SlotButtonKinovaSwitch(){

    if(!mpc_kinova->IsKinovaInitialized()){
        if(!mpc_kinova->InitKinova())
            QMessageBox::information(this, tr("Fail to Connect KINOVA"), tr("Check KINOVA"));
        else{
            ui->bt_kinova->setText("Kinova Close");
            mpc_kinova->KinovaInitMotion();
            sleep(2);
            mpc_kinova->KinovaAlignToPanel();
        }
    }
    else{
        mpc_kinova->CloseKinova();
        ui->bt_kinova->setText("Kinova On");
    }
}

void EurecarURV_Dlg::SlotButtonKinovaReset(){
    if(!mpc_kinova->IsKinovaInitialized()){
        if(!mpc_kinova->InitKinova())
            QMessageBox::information(this, tr("Fail to Connect KINOVA"), tr("Check KINOVA"));
        else{
            mpc_kinova->KinovaResetFactorial();
        }
    }
}

void EurecarURV_Dlg::SlotButtonLRFManiSwitch(){

    QString dev_path;
    dev_path = ui->ed_mani_lrf_ip->text();

    QByteArray char_dev_path = dev_path.toLocal8Bit();

    if(!mpc_mani__lrf->IsLRFOn()){
        if(!mpc_mani__lrf->InitLRF(char_dev_path.data(), UST_20LX)){
            QMessageBox::information(this, tr("Fail to Open Device"), tr("Check the KINOVA LRF"));
            return;
        }
        ui->bt_mani_lrf->setText("K LRF Off");
    }
    else{
        mpc_mani__lrf->CloseLRF();
        ui->bt_mani_lrf->setText("Kinova LRF");
    }
}

void EurecarURV_Dlg::SlotButtonLRFDriveSwitch(){

    QString dev_path;
    dev_path = ui->ed_drive_lrf_path->text();

    QByteArray char_dev_path = dev_path.toLocal8Bit();

    if(!mpc_drive_lrf->IsLRFOn()){
        if(!mpc_drive_lrf->InitLRF(char_dev_path.data(), UTM_30LX)){
            QMessageBox::information(this, tr("Fail to Open Device"), tr("Check the UGV LRF"));
            return;
        }
        ui->bt_drive_lrf->setText("V LRF Off");
    }
    else{
        mpc_drive_lrf->CloseLRF();
        ui->bt_drive_lrf->setText("UGV LRF");
    }
}

void EurecarURV_Dlg::SlotButtonGPSSwitch(){
    QString dev_path;
    dev_path = ui->ed_gps_path->text();

    QByteArray char_dev_path = dev_path.toLocal8Bit();

    if(!mpc_gps->port->isOpen()){
        if(!mpc_gps->GpsInit(ui->ed_gps_path->text().toStdString())){
            QMessageBox::information(this, tr("Fail to Open Device"), tr("Check the GPS"));
            return;
        }
        ui->bt_gps->setText("GPS OFF");
    }
    else{
        mpc_gps->GpsClose();
        ui->bt_gps->setText("GPS Connect");
    }
}

void EurecarURV_Dlg::SlotButtonGPSInitPosSwitch(){
    mpc_gps->SetInitGPS();
}

void EurecarURV_Dlg::SlotButtonVehicleSwitch(){

    QString dev_path;
    dev_path = ui->ed_vehicle_path->text();

    QByteArray char_dev_path = dev_path.toLocal8Bit();

    if(!mpc_vehicle->IsConnected()){
        if(!mpc_vehicle->Connect(char_dev_path.data())){
            QMessageBox::information(this, tr("Fail to Open Device"), tr("Check the UGV"));
            return;
        }
        ui->bt_ugv->setText("UGV OFF");
    }
    else{
        mpc_vehicle->Disconnect();
        ui->bt_ugv->setText("UGV Connect");
    }
}

void EurecarURV_Dlg::SlotButtonCameraSwitch(){

}

void EurecarURV_Dlg::SlotButtonMissionRun(){

    if(mpc_script->IsMissionPaused()){
        QMessageBox::information(this, tr("Fail to Start Mission"), tr("Mission Termiante First!!"));
        return;
    }

    SCRIPT_PLAYER_OPTION player_option;

    int mission_num = ui->ed_script_mission_s->text().toInt();

    player_option.start_mission_num = mission_num;
    player_option.end_mission_num = mission_num;

    player_option.start_step_num = 0;
    player_option.end_step_num = 100;//Dummy Value

    mpc_script->SetPlayerOption(player_option);

    mpc_script->SetMissionPause(false);
    mpc_script->SetMissionTerminate(false);

    ui->bt_mission_pause->setText("Pause");
    return;
}

void EurecarURV_Dlg::SlotButtonMissionPartialRun(){

    if(mpc_script->IsMissionPaused()){
        QMessageBox::information(this, tr("Fail to Start Mission"), tr("Mission Termiante First!!"));
        return;
    }

    SCRIPT_PLAYER_OPTION player_option;

    int mission_num = ui->ed_script_mission_s->text().toInt();

    player_option.start_mission_num = mission_num;
    player_option.end_mission_num = mission_num;//Dummy Value

    player_option.start_step_num = ui->ed_script_step_s->text().toInt();
    player_option.end_step_num = ui->ed_script_step_f->text().toInt();//Dummy Value

    if(player_option.start_step_num > player_option.end_step_num)
        player_option.end_step_num = player_option.start_step_num;

    mpc_script->SetPlayerOption(player_option);

    mpc_script->SetMissionPause(false);
    mpc_script->SetMissionTerminate(false);

    ui->bt_mission_pause->setText("Pause");
    return;

}

void EurecarURV_Dlg::SlotButtonScenarioPartialRun(){

    SCRIPT_PLAYER_OPTION player_option;

    int mission_num_s = ui->ed_script_mission_s->text().toInt();
    int mission_num_f = ui->ed_script_mission_f->text().toInt();

    if(mission_num_s > mission_num_f)
        mission_num_f = mission_num_s;

    player_option.start_mission_num = mission_num_s;
    player_option.end_mission_num = mission_num_f;

    player_option.start_step_num = 0;
    player_option.end_step_num = 100;//Dummy Value

    mpc_script->SetPlayerOption(player_option);

    mpc_script->SetMissionPause(false);
    mpc_script->SetMissionTerminate(false);
    return;
}

//-------------------------------------------------
// SystemMessage Update
//-------------------------------------------------
void EurecarURV_Dlg::SlotViewSystemMessage(QString _message){

    ui->msg_urv_status->append(_message);
}

//ListView Update
void EurecarURV_Dlg::SlotMissionListUpdate(QModelIndex _q_mode_index){

    int r = _q_mode_index.row();
    QStringList step_title_list;

    ui->ed_script_mission_s->setText(QString::number(r));
    ui->ed_script_mission_f->setText(QString::number(r+1));

    ui->ed_script_step_s->setText(QString::number(0));
    ui->ed_script_step_f->setText(QString::number(0));

    mpc_script->GetStepTitle(r, step_title_list);

    ui->cb_step_title->clear();
    ui->cb_step_title->addItems(step_title_list);

    return;
}

void EurecarURV_Dlg::SlotStepListUpdate(int _index){

    ui->ed_script_step_s->setText(QString::number(_index));

    return;
}

// Menu Button
void EurecarURV_Dlg::SlotMenuButtonDriving_Dlg(){

    if(mpdlg_driving->isVisible()){
        QMessageBox::information(this, tr("Fail to Open Dialog"), tr("Isn't it already open?"));
        return;
    }

    QWidget *pwidget = NULL;

    foreach(pwidget,QApplication::topLevelWidgets()){
        if((pwidget->isWindow()) && pwidget->isModal() ){
            mpdlg_driving->setParent(pwidget);
        }
    }

    if(pwidget == NULL){
        mpdlg_driving->setParent(this);
    }

    mpdlg_driving->show();
    mpdlg_driving->activateWindow();
    mpdlg_driving->activateWindow();
    mpdlg_driving->setFocus();

}

void EurecarURV_Dlg::SlotMenuButtonManipulation_Dlg(){
    if(mpdlg_manipulation->isVisible()){
        QMessageBox::information(this, tr("Fail to Open Dialog"), tr("Isn't it already open?"));
        return;
    }

    QWidget *pwidget = NULL;

    foreach(pwidget,QApplication::topLevelWidgets()){
        if((pwidget->isWindow()) && pwidget->isModal() ){
            mpdlg_manipulation->setParent(pwidget);
        }
    }

    if(pwidget == NULL){
        mpdlg_manipulation->setParent(this);
    }

    mpdlg_manipulation->show();
    mpdlg_manipulation->activateWindow();
    mpdlg_manipulation->activateWindow();
    mpdlg_manipulation->setFocus();
}

void EurecarURV_Dlg::SlotMenuButtonScenarioLoad(){

    QFileDialog dialog(this);
    QStringList fileName;

    dialog.setDirectory("/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/Script");

    dialog.setFileMode(QFileDialog::AnyFile);

    if(dialog.exec())
        fileName = dialog.selectedFiles();
    else
        return;

    if(!mpc_script->InterpreteScenarioScriptFile(fileName[0]))
        QMessageBox::information(this, tr("Fail to Open Scenario"), tr("Check the Scenario File"));

    //Scenario Info Display
    ScriptInfoDisplay();
}

void EurecarURV_Dlg::SlotButtonScenarioLoad(){

    QString file_path = "/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/Script/Scenario.md";

    if(!mpc_script->InterpreteScenarioScriptFile(file_path))
        QMessageBox::information(this, tr("Fail to Open Scenario"), tr("Check the Scenario File"));

    ScriptInfoDisplay();
}

void EurecarURV_Dlg::SlotButtonMissionPause(){

    if(!mpc_script->IsMissionPaused()){
        mpc_script->SetMissionPause(true);
        ui->bt_mission_pause->setText("Resume");
    }
    else{
        mpc_script->SetMissionPause(false);
        ui->bt_mission_pause->setText("Pause");
    }

}

void EurecarURV_Dlg::SlotButtonMissionTerminate(){

    mpc_script->SetMissionTerminate(true);
    mpc_script->SetMissionPause(false);
    ui->bt_mission_pause->setText("Pause");
}

void EurecarURV_Dlg::ScriptInfoDisplay(){

    ui->lsview_mission_title->clear();

    SCENARIO_SCRIPT scenario_script;
    QStringList mission_list;

    mpc_script->GetScenarioScript(scenario_script);

    for(unsigned int i = 0; i < scenario_script.mission_file_name.size(); i++){
        mission_list << scenario_script.mission_file_name.at(i);
    }

    ui->lsview_mission_title->addItems(mission_list);
    ui->lsview_mission_title->update();

    return;
}

void EurecarURV_Dlg::SlotVeloyneParser(bool _parser_complete){
    if(_parser_complete){
        ui->qvtk_velodyne_main_dlg->update();
    }
    if((mpc_lms511->IsLMS511Init()) && (!mpc_lms511->isRunning()))
    {
        mpc_lms511->start();
    }
}
