#include "EurecarURV_Dlg.h"
#include "ui_EurecarURV_Dlg.h"

EurecarURV_Dlg::EurecarURV_Dlg(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EurecarURV_Dlg)
{
    ui->setupUi(this);

    //-------------------------------------------------
    // Device Class Initialize
    //-------------------------------------------------
    mpc_gps = new CGPS;
    mpc_lrf = new CLRF;
    mpc_camera = new CCamera;
    mpc_kinova = new CKinova;
    mpc_vehicle = new CVehicle;
    mpc_velodyne = new CVelodyne;

    //-------------------------------------------------
    // Script Class Initialize
    //-------------------------------------------------
    mpc_script = new CScript(mpc_gps, mpc_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne);

    //-------------------------------------------------
    // Mission Class Initialize
    //-------------------------------------------------

    //Driving Class Initialize
    mpc_drivig = new CDriving(mpc_gps, mpc_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne);

    //Manipulation Class Initialize
    mpc_manipulation = new CManipulation(mpc_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne);

    //-------------------------------------------------
    // SubDialog Initialize
    //-------------------------------------------------

    //Driving Dialog Box Initialize
    mdlg_driving.InitDlg(mpc_drivig);

    //Manipulation Dialog Box Initialize
    mdlg_manipulation.InitDlg(mpc_manipulation);

    //-------------------------------------------------
    // Connection
    //-------------------------------------------------

    //Menu Button Connect
    connect(ui->action_driving_dialog_box,SIGNAL(triggered()),this,SLOT(SlotMenuButtonDriving_Dlg()));
    connect(ui->action_manipulation_dialog_box,SIGNAL(triggered()),this,SLOT(SlotMenuButtonManipulation_Dlg()));
    //Push Button Connect
    connect(ui->bt_velodyne,SIGNAL(clicked()), this, SLOT(SlotButtonVelodyne()));


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
        ui->bt_velodyne->setText("Velodyne Off");
    }
    else{
        mpc_velodyne->SetVelodyneThread(false);
        ui->bt_velodyne->setText("Velodyne On");
    }
}

// Menu Button
void EurecarURV_Dlg::SlotMenuButtonDriving_Dlg(){

    if(mdlg_driving.isVisible()){
        QMessageBox::information(this, tr("Fail to Open Dialog"), tr("Isn't it already open?"));
        return;
    }

    QWidget *pwidget = NULL;

    foreach(pwidget,QApplication::topLevelWidgets()){
        if((pwidget->isWindow()) && pwidget->isModal() ){
            mdlg_driving.setParent(pwidget);
        }
    }

    if(pwidget == NULL){
        mdlg_driving.setParent(this);
    }

    mdlg_driving.show();
    mdlg_driving.activateWindow();
    mdlg_driving.activateWindow();
    mdlg_driving.setFocus();

}

void EurecarURV_Dlg::SlotMenuButtonManipulation_Dlg(){
    if(mdlg_manipulation.isVisible()){
        QMessageBox::information(this, tr("Fail to Open Dialog"), tr("Isn't it already open?"));
        return;
    }

    QWidget *pwidget = NULL;

    foreach(pwidget,QApplication::topLevelWidgets()){
        if((pwidget->isWindow()) && pwidget->isModal() ){
            mdlg_manipulation.setParent(pwidget);
        }
    }

    if(pwidget == NULL){
        mdlg_manipulation.setParent(this);
    }

    mdlg_manipulation.show();
    mdlg_manipulation.activateWindow();
    mdlg_manipulation.activateWindow();
    mdlg_manipulation.setFocus();
}
