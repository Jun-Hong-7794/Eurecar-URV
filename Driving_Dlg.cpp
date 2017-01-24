#include "Driving_Dlg.h"
#include "ui_Driving_Dlg.h"

Driving_Dlg::Driving_Dlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Driving_Dlg)
{
    VTK_MODULE_INIT(vtkRenderingOpenGL2);
    ui->setupUi(this);

    qRegisterMetaType<QString>("QString");
    qRegisterMetaType<cv::Mat>("cv::Mat");

    //-------------------------------------------------
    // Connection
    //-------------------------------------------------

    //Button Connect
    connect(ui->bt_vehicle_run,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleOperate()));
    connect(ui->bt_vehicle_connection,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleConnet()));
    connect(ui->bt_vehicle_set_option,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleOptionSetting()));

    connect(ui->bt_velodyne_connection,SIGNAL(clicked()),this,SLOT(SlotButtonVelodyneConnet()));
}

Driving_Dlg::Driving_Dlg(CDriving* _pc_driving,QWidget *parent):
    QDialog(parent),
    ui(new Ui::Driving_Dlg)
{
    VTK_MODULE_INIT(vtkRenderingOpenGL2);
    ui->setupUi(this);

    qRegisterMetaType<QString>("QString");
    qRegisterMetaType<cv::Mat>("cv::Mat");

    //-------------------------------------------------
    // Connection
    //-------------------------------------------------

    //Button Connect
    connect(ui->bt_vehicle_run,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleOperate()));
    connect(ui->bt_vehicle_connection,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleConnet()));
    connect(ui->bt_vehicle_set_option,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleOptionSetting()));

    connect(ui->bt_velodyne_connection,SIGNAL(clicked()),this,SLOT(SlotButtonVelodyneConnet()));

    //Driving Class Init
    mpc_drivig = _pc_driving;

    //PCL Init
    mpc_drivig->PCLInit();
    ui->qvtk_velodyne_driving_dlg->SetRenderWindow(mpc_drivig->GetPCL()->viewer->getRenderWindow());
    (mpc_drivig->GetPCL())->viewer->setupInteractor(ui->qvtk_velodyne_driving_dlg->GetInteractor(),ui->qvtk_velodyne_driving_dlg->GetRenderWindow());
    (mpc_drivig->GetPCL())->viewer->setBackgroundColor(0,0,0);
    (mpc_drivig->GetPCL())->viewer->addCoordinateSystem(1.0);
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->cloud);
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,1,"cloud");

    // Velodyne View Connect
    connect(mpc_drivig,SIGNAL(SignalVelodyneParser(bool)),this,SLOT(SlotVeloyneParser(bool)));

}


Driving_Dlg::~Driving_Dlg()
{
    delete ui;
}

void Driving_Dlg::InitDlg(CDriving* _p_driving){

    //Driving Class Init
    mpc_drivig = _p_driving;

    //PCL Init
    mpc_drivig->PCLInit();
    ui->qvtk_velodyne_driving_dlg->SetRenderWindow(mpc_drivig->GetPCL()->viewer->getRenderWindow());
    (mpc_drivig->GetPCL())->viewer->setupInteractor(ui->qvtk_velodyne_driving_dlg->GetInteractor(),ui->qvtk_velodyne_driving_dlg->GetRenderWindow());
    (mpc_drivig->GetPCL())->viewer->setBackgroundColor(0,0,0);
    (mpc_drivig->GetPCL())->viewer->addCoordinateSystem(1.0);
    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->cloud);
    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,1,"cloud");

    // Velodyne View Connect
    connect(mpc_drivig,SIGNAL(SignalVelodyneParser(bool)),this,SLOT(SlotVeloyneParser(bool)));

}


//-------------------------------------------------
// Button
//-------------------------------------------------

void Driving_Dlg::SlotButtonVelodyneConnet(){

    if(!mpc_drivig->IsVelodyneConnected()){
        if(!mpc_drivig->ConnectVelodyne())
            QMessageBox::information(this, tr("Fail to Connect Velodyne"), tr("Check Velodyne"));

        ui->bt_velodyne_connection->setText("Disconnect");
    }
    else{
        mpc_drivig->CloseVelodyne();
        ui->bt_velodyne_connection->setText("Connect");
    }
}

void Driving_Dlg::SlotButtonVehicleConnet(){
    if(!mpc_drivig->ConnectVehicle())
        QMessageBox::information(this, tr("Fail to Connect Vehicle"), tr("Check Vehicle Status"));
}

void Driving_Dlg::SlotButtonVehicleOperate(){

    if(!mpc_drivig->isRunning()){
        if(!mpc_drivig->ConnectVehicle())
            QMessageBox::information(this, tr("Fail to Connect Vehicle"), tr("Check Vehicle Status"));

        DRIVING_STRUCT driving_struct;

        if(ui->rd_vehicle_dir_forward->isChecked()){
            driving_struct.direction = UGV_move_forward;
        }
        else if(ui->rd_vehicle_dir_backward->isChecked()){
            driving_struct.direction = UGV_move_backward;
        }
        else if(ui->rd_vehicle_dir_left->isChecked()){
            driving_struct.direction = UGV_move_left;
        }
        else if(ui->rd_vehicle_dir_right->isChecked()){
            driving_struct.direction = UGV_move_right;
        }

        driving_struct.velocity = ui->ed_vehicle_velocity->text().toInt();

        driving_struct.driving_mission = true;

        mpc_drivig->SetDrivingOption(driving_struct);

        mpc_drivig->SelectMainFunction(DRIVE_INX_DRIVE_TO_PANEL);

        ui->bt_vehicle_run->setText("Stop");
    }
    else{
        DRIVING_STRUCT driving_struct;

        driving_struct.direction = 0;
        driving_struct.velocity = 0;
        driving_struct.driving_mission = false;

        mpc_drivig->SetDrivingOption(driving_struct);

        ui->bt_vehicle_run->setText("Go");
    }
}

void Driving_Dlg::SlotButtonVehicleOptionSetting(){

    DRIVING_STRUCT driving_struct;

    if(ui->rd_vehicle_dir_forward->isChecked()){
        driving_struct.direction = UGV_move_forward;
    }
    else if(ui->rd_vehicle_dir_backward->isChecked()){
        driving_struct.direction = UGV_move_backward;
    }
    else if(ui->rd_vehicle_dir_left->isChecked()){
        driving_struct.direction = UGV_move_left;
    }
    else if(ui->rd_vehicle_dir_right->isChecked()){
        driving_struct.direction = UGV_move_right;
    }

    driving_struct.velocity = ui->ed_vehicle_velocity->text().toInt();

    mpc_drivig->SetDrivingOption(driving_struct);

    return;
}

void Driving_Dlg::SlotVeloyneParser(bool _parser_complete){
    if(_parser_complete){
        ui->qvtk_velodyne_driving_dlg->update();
    }
}









