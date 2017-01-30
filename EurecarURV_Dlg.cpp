#include "EurecarURV_Dlg.h"
#include "ui_EurecarURV_Dlg.h"

EurecarURV_Dlg::EurecarURV_Dlg(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EurecarURV_Dlg)
{
    ui->setupUi(this);

    qRegisterMetaType<CartesianPosition>("CartesianPosition");
    qRegisterMetaType<LRF_VEHICLE_STRUCT>("LRF_VEHICLE_STRUCT");

    //-------------------------------------------------
    // Device Class Initialize
    //-------------------------------------------------
    mpc_gps = new CGPS;
    mpc_lrf = new CLRF;
    mpc_camera = new CCamera;
    mpc_kinova = new CKinova;
    mpc_vehicle = new CVehicle;
    mpc_velodyne = new CVelodyne;
    mpc_gripper = new CGripper;


    //-------------------------------------------------
    // Mission Class Initialize
    //-------------------------------------------------

    //Driving Class Initialize
    mpc_drivig = new CDriving(mpc_gps, mpc_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne);

    //Manipulation Class Initialize
    mpc_manipulation = new CManipulation(mpc_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne, mpc_gripper);

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
    connect(ui->bt_velodyne,SIGNAL(clicked()), this, SLOT(SlotButtonVelodyne()));
    connect(ui->bt_kinova,SIGNAL(clicked()), this, SLOT(SlotButtonKinova()));
    connect(ui->bt_mission_run,SIGNAL(clicked()), this, SLOT(SlotButtonMissionRun()));

    connect(ui->bt_mission_partial_run,SIGNAL(clicked()), this, SLOT(SlotButtonMissionPartialRun()));
    connect(ui->bt_scenario_partial_run,SIGNAL(clicked()), this, SLOT(SlotButtonScenarioPartialRun()));

    //Click List View
    connect(ui->lsview_mission_title,SIGNAL(clicked(QModelIndex)), this, SLOT(SlotMissionListUpdate(QModelIndex)));
    //Click Combo Box
    connect(ui->cb_step_title,SIGNAL(activated(int)), this, SLOT(SlotStepListUpdate(int)));

    //View System Message
    connect(mpc_script,SIGNAL(SignalScriptMessage(QString)), this, SLOT(SlotViewSystemMessage(QString)));

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
    delete mpc_lrf;
    delete mpc_camera;
    delete mpc_kinova;
    delete mpc_vehicle;
    delete mpc_velodyne;

    delete ui;
}

//-------------------------------------------------
// Button
//-------------------------------------------------

void EurecarURV_Dlg::SlotButtonVelodyne(){

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

void EurecarURV_Dlg::SlotButtonKinova(){

    if(!mpc_kinova->IsKinovaInitialized()){
        if(!mpc_kinova->InitKinova())
            QMessageBox::information(this, tr("Fail to Connect KINOVA"), tr("Check KINOVA"));
        else{
            ui->bt_kinova->setText("Kinova Close");
            mpc_kinova->KinovaInitMotion();
        }
    }
    else{
        mpc_kinova->CloseKinova();
        ui->bt_kinova->setText("Kinova On");
    }
}

void EurecarURV_Dlg::SlotButtonMissionRun(){

    SCRIPT_PLAYER_OPTION player_option;

    int mission_num = ui->ed_script_mission_s->text().toInt();

    player_option.start_mission_num = mission_num;
    player_option.end_mission_num = mission_num;

    player_option.start_step_num = 0;
    player_option.end_step_num = 100;//Dummy Value

    mpc_script->SetPlayerOption(player_option);
    return;
}

void EurecarURV_Dlg::SlotButtonMissionPartialRun(){

    SCRIPT_PLAYER_OPTION player_option;

    int mission_num = ui->ed_script_mission_s->text().toInt();

    player_option.start_mission_num = mission_num;
    player_option.end_mission_num = mission_num;//Dummy Value

    player_option.start_step_num = ui->ed_script_step_s->text().toInt();
    player_option.end_step_num = ui->ed_script_step_f->text().toInt();//Dummy Value

    if(player_option.start_step_num > player_option.end_step_num)
        player_option.end_step_num = player_option.start_step_num;

    mpc_script->SetPlayerOption(player_option);
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
    return;
}

//SystemMessage Update
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





