#include "Manipulation_Dlg.h"
#include "ui_Manipulation_Dlg.h"

Manipulation_Dlg::Manipulation_Dlg(CManipulation* _pc_manipulation, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Manipulation_Dlg){

    ui->setupUi(this);

    mpc_manipulation = _pc_manipulation;

    //Graphic Scene Initialize
    int lrf_view_width  = ui->view_lrf->geometry().width();
    int lrf_view_height = ui->view_lrf->geometry().height();

    mp_lrf_image_grahicscene = new QGraphicsScene(QRectF(0, 0, lrf_view_width, lrf_view_height), 0);

    int camera_view_width  = ui->view_camera->geometry().width();
    int camera_view_height = ui->view_camera->geometry().height();

    mp_camera_image_grahicscene = new QGraphicsScene(QRectF(0, 0, camera_view_width, camera_view_height), 0);

    int segnet_view_width  = ui->view_segnet->geometry().width();
    int segnet_view_height = ui->view_segnet->geometry().height();

    mp_segnet_image_grahicscene = new QGraphicsScene(QRectF(0, 0, segnet_view_width, segnet_view_height), 0);


    //Button Connetion
    connect(ui->bt_kinova_init, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaInit()));

    connect(ui->bt_kinova_init_motion, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaInitMotion()));
    connect(ui->bt_kinova_align, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaAlignPanel()));
    connect(ui->bt_kinova_manipulate, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaDoManipulate()));

    connect(ui->bt_kinova_up, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepUp()));
    connect(ui->bt_kinova_down, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepDw()));
    connect(ui->bt_kinova_right, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepRi()));
    connect(ui->bt_kinova_left, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepLe()));
    connect(ui->bt_kinova_forward, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepFw()));
    connect(ui->bt_kinova_backward, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepBw()));

    connect(ui->bt_kinova_roll, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepRollUp()));
    connect(ui->bt_kinova_roll_dw, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepRollDw()));
    connect(ui->bt_kinova_pitch_up, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepPitchUp()));
    connect(ui->bt_kinova_pitch_dw, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepPitchDw()));

    connect(ui->bt_lrf_kinova_ctrl, SIGNAL(clicked()), this, SLOT(SlotButtonLRFKinovaCtrl()));
    connect(ui->bt_kinova_force_ctrl, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaForceCtrl()));
    connect(ui->bt_gripper_force_ctrl, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorLoadCheckIter()));
    connect(ui->bt_lrf_horizent_distance, SIGNAL(clicked()), this, SLOT(SlotButtonHorizenDistance()));


    connect(ui->bt_end_effector_grasp, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorGrasp()));
    connect(ui->bt_end_effector_present_pos_check, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorPoseCheck()));
    connect(ui->bt_end_effector_org_pos, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorGoToOrg()));

    connect(ui->bt_lrf_on, SIGNAL(clicked()), this, SLOT(SlotButtonLRFOn()));
    connect(ui->bt_camera_on, SIGNAL(clicked()), this, SLOT(SlotButtonCameraOn()));

    connect(ui->bt_lrf_get_info, SIGNAL(clicked()), this, SLOT(SlotButtonGetLRFInfo()));
    connect(ui->bt_kinova_get_pose, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaGetPosition()));

    connect(mpc_manipulation, SIGNAL(SignalKinovaPosition(CartesianPosition)), this, SLOT(SlotEditeKinovaPosition(CartesianPosition)));
    connect(mpc_manipulation, SIGNAL(SignalKinovaForceVector(CartesianPosition)), this, SLOT(SlotEditeKinovaForceVector(CartesianPosition)));
    connect(mpc_manipulation, SIGNAL(SignalLRFHorizentDistance(LRF_VEHICLE_STRUCT)), this, SLOT(SlotLRFHorizentDistance(LRF_VEHICLE_STRUCT)));

    connect(mpc_manipulation, SIGNAL(SignalLRFHorizentDistance(LRF_VEHICLE_STRUCT)), this, SLOT(SlotLRFHorizentDistance(LRF_VEHICLE_STRUCT)));

    //Check Button
    connect(ui->ck_segnet_switch, SIGNAL(clicked(bool)), this, SLOT(SlotButtonSegnetOn(bool)));

    //View Connetion
    connect(mpc_manipulation, SIGNAL(SignalLRFImage(cv::Mat)), this, SLOT(SlotViewLRFImage(cv::Mat)));
    connect(mpc_manipulation, SIGNAL(SignalCameraImage(cv::Mat)), this, SLOT(SlotViewCameraImage(cv::Mat)));
    connect(mpc_manipulation, SIGNAL(SignalSegnetImage(cv::Mat)), this, SLOT(SlotViewSegnetImage(cv::Mat)));

}

Manipulation_Dlg::~Manipulation_Dlg()
{
    delete ui;
}

//-------------------------------------------------
// Button
//-------------------------------------------------

// KINOVA
void Manipulation_Dlg::SlotButtonKinovaInit(){

    if(!mpc_manipulation->InitKinova())
        QMessageBox::information(this, tr("Fail to Init Kinova"), tr("Check Kinova Status"));

    return;
}

void Manipulation_Dlg::SlotButtonKinovaInitMotion(){

    if(!mpc_manipulation->KinovaInitMotion())
        QMessageBox::information(this, tr("Fail to InitMotion"), tr("Check Kinova Status"));

    return;
}

void Manipulation_Dlg::SlotButtonKinovaAlignPanel(){

    if(!mpc_manipulation->KinovaAlignPanel())
        QMessageBox::information(this, tr("Fail to Align Panel"), tr("Check Kinova Status"));

    return;
}

void Manipulation_Dlg::SlotButtonKinovaDoManipulate(){

    CartesianPosition position;

    position.Coordinates.X = ui->ed_kinova_x->text().toFloat();
    position.Coordinates.Y = ui->ed_kinova_y->text().toFloat();
    position.Coordinates.Z = ui->ed_kinova_z->text().toFloat();

    position.Coordinates.ThetaZ = ui->ed_kinova_roll->text().toFloat();
    position.Coordinates.ThetaY = ui->ed_kinova_pitch->text().toFloat();
    position.Coordinates.ThetaX = ui->ed_kinova_yaw->text().toFloat();

    if(!mpc_manipulation->KinovaDoManipulate(position))
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepUp(){

    if(!mpc_manipulation->KinovaMoveUnitStepUp())
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepDw(){

    if(!mpc_manipulation->KinovaMoveUnitStepDw())
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepRi(){

    if(!mpc_manipulation->KinovaMoveUnitStepRi())
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepLe(){

    if(!mpc_manipulation->KinovaMoveUnitStepLe())
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepFw(){
    if(!mpc_manipulation->KinovaMoveUnitStepFw())
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepBw(){
    if(!mpc_manipulation->KinovaMoveUnitStepBw())
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepRollUp(){

    double unit_step = ui->ed_kinova_unit_step->text().toDouble();

    if(!mpc_manipulation->KinovaMoveUnitStep(0, 0, 0, 0, 0, unit_step))
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepRollDw(){

    double unit_step = ui->ed_kinova_unit_step->text().toDouble();

    if(!mpc_manipulation->KinovaMoveUnitStep(0, 0, 0, 0, 0, (-1)*unit_step))
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepPitchUp(){

    double unit_step = ui->ed_kinova_unit_step->text().toDouble();

    if(!mpc_manipulation->KinovaMoveUnitStep(0, 0, 0, 0, unit_step, 0))
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotButtonKinovaMoveStepPitchDw(){

    double unit_step = ui->ed_kinova_unit_step->text().toDouble();

    if(!mpc_manipulation->KinovaMoveUnitStep(0, 0, 0, 0, (-1)*unit_step, 0))
        QMessageBox::information(this, tr("Fail to Move Step"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotEditeKinovaPosition(CartesianPosition _position){

    ui->ed_kinova_x->setText(QString::number(_position.Coordinates.X, 'f', 4));
    ui->ed_kinova_y->setText(QString::number(_position.Coordinates.Y, 'f', 4));
    ui->ed_kinova_z->setText(QString::number(_position.Coordinates.Z, 'f', 4));

    ui->ed_kinova_roll->setText(QString::number(_position.Coordinates.ThetaZ, 'f', 4));
    ui->ed_kinova_pitch->setText(QString::number(_position.Coordinates.ThetaY, 'f', 4));
    ui->ed_kinova_yaw->setText(QString::number(_position.Coordinates.ThetaX, 'f', 4));

    return;
}

void Manipulation_Dlg::SlotEditeKinovaForceVector(CartesianPosition _force_vector){

    ui->ed_kinova_force_x->setText(QString::number(_force_vector.Coordinates.X, 'f', 4));
    ui->ed_kinova_force_y->setText(QString::number(_force_vector.Coordinates.Y, 'f', 4));
    ui->ed_kinova_force_z->setText(QString::number(_force_vector.Coordinates.Z, 'f', 4));

    ui->ed_kinova_force_roll->setText(QString::number(_force_vector.Coordinates.ThetaZ, 'f', 4));
    ui->ed_kinova_force_pitch->setText(QString::number(_force_vector.Coordinates.ThetaY, 'f', 4));
    ui->ed_kinova_force_yaw->setText(QString::number(_force_vector.Coordinates.ThetaX, 'f', 4));

    return;
}

void Manipulation_Dlg::SlotLRFHorizentDistance(LRF_VEHICLE_HORIZEN_STRUCT _lrf_horizen){

    ui->ed_lrf_horizen_s_deg->setText(QString::number(_lrf_horizen.s_inlier_deg, 'f', 4));
    ui->ed_lrf_horizen_e_deg->setText(QString::number(_lrf_horizen.e_inlier_deg, 'f', 4));
    ui->ed_lrf_horizen_distance->setText(QString::number(_lrf_horizen.horizen_distance, 'f', 4));

    return;
}

void Manipulation_Dlg::SlotButtonKinovaForceCtrl(){

    KINOVA_FORCE_CTRL_STRUCT kinova_force_ctrl_option;

    kinova_force_ctrl_option.kinova_force_ctrl_mission = true;
    kinova_force_ctrl_option.step_count = ui->ed_kinova_force_step_count->text().toDouble();
    kinova_force_ctrl_option.forece_threshold = ui->ed_kinova_force_threshold->text().toDouble();

    mpc_manipulation->SetManipulationOption(kinova_force_ctrl_option);

    if(!mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_FORCE_CLRL))
        QMessageBox::information(this, tr("Fail to Operate LRF Kinova"), tr("Check LRF Or Kinova"));

}

void Manipulation_Dlg::SlotButtonKinovaGetPosition(){

    CartesianPosition position;

    mpc_manipulation->KinovaGetPosition(position);

    SlotEditeKinovaPosition(position);
}

//LRF-Kinova
void Manipulation_Dlg::SlotButtonLRFKinovaCtrl(){

    LRF_KINOVA_STRUCT lrf_kinova_option;

    lrf_kinova_option.desired_distance = ui->ed_lrf_kinova_desired_dst->text().toDouble();
    lrf_kinova_option.error = ui->ed_lrf_kinova_error->text().toDouble();
    lrf_kinova_option.s_deg = ui->ed_lrf_kinova_s_deg->text().toDouble();
    lrf_kinova_option.e_deg = ui->ed_lrf_kinova_e_deg->text().toDouble();

    mpc_manipulation->SetManipulationOption(lrf_kinova_option);

    if(!mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_KINOVA))
        QMessageBox::information(this, tr("Fail to Operate LRF Kinova"), tr("Check LRF Or Kinova"));
}


//Camera
void Manipulation_Dlg::SlotButtonCameraOn(){
    if(!mpc_manipulation->IsCameraConnected()){
        if(!mpc_manipulation->InitCamera()){
            QMessageBox::information(this, tr("Fail to Connect Camera"), tr("Check Camera Status"));
            return;
        }
        else{
            ui->bt_camera_on->setText("Off");
        }
    }
    else{
        mpc_manipulation->CloseCamera();
        ui->bt_camera_on->setText("On");
    }
}

void Manipulation_Dlg::SlotButtonSegnetOn(bool _check){

    if(_check){
        if(!mpc_manipulation->IsCameraConnected()){
            QMessageBox::information(this, tr("Fail to Connect Camera"), tr("Check Camera Status"));
        }
        if(!mpc_manipulation->SetRGBDFunction(THREAD_SEGNET_INDEX))
            QMessageBox::information(this, tr("Fail to Run Segnet"), tr("Check Camera Status"));;
    }
    else{
        mpc_manipulation->SetRGBDFunction(0);
    }
}

// LRF
void Manipulation_Dlg::SlotButtonLRFOn(){
    if(!mpc_manipulation->IsLRFConnected()){
        if(!mpc_manipulation->InitLRF()){
            QMessageBox::information(this, tr("Fail to Connect LRF"), tr("Check LRF Status"));
            return;
        }
        else{
            ui->bt_lrf_on->setText("Off");
        }
    }
    else{
        if(!mpc_manipulation->CloseLRF()){
            QMessageBox::information(this, tr("Fail to Close LRF"), tr("Check LRF Status"));
            return;
        }
        ui->bt_lrf_on->setText("On");
    }
}

void Manipulation_Dlg::SlotButtonGetLRFInfo(){

    double slope = 0;
    double distance = 0;

    if(!mpc_manipulation->GetLRFInfo(slope, distance, 30, 150)){
        QMessageBox::information(this, tr("Fail to Get LRF Info"), tr("Check LRF Status"));
        return;
    }
    else{
        ui->ed_lrf_slope->setText(QString::number(slope, 'f', 3));
        ui->ed_lrf_distance->setText(QString::number(distance, 'f', 3));
    }
}

// End Effector
void Manipulation_Dlg::SlotButtonEEffectorGrasp(){

    if(!mpc_manipulation->InitGripper()){
        QMessageBox::information(this, tr("Fail to Init Gripper"), tr("Check Gripper Status"));
        return;
    }

    double rel_deg = ui->ed_end_effector_deg->text().toDouble();

    if(!mpc_manipulation->GripperGoRelPose(rel_deg)){
        QMessageBox::information(this, tr("Fail to Rel pose"), tr("Check Gripper Status"));
        return;
    }
}

void Manipulation_Dlg::SlotButtonEEffectorPoseCheck(){

    uint16_t present_pos = 0;

    if(!mpc_manipulation->GripperPresentPose(present_pos)){
        QMessageBox::information(this, tr("Fail to Get pose"), tr("Check Gripper Status"));
        return;
    }

    ui->ed_end_effector_present_pos->setText(QString::number(present_pos));
}

void Manipulation_Dlg::SlotButtonEEffectorGoToOrg(){

    uint16_t org_pos = 13;

    if(!mpc_manipulation->GripperGoThePose(org_pos)){
        QMessageBox::information(this, tr("Fail to Get pose"), tr("Check Gripper Status"));
        return;
    }

}

void Manipulation_Dlg::SlotButtonEEffectorLoadCheckIter(){

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_ctrl;

    gripper_force_ctrl = mpc_manipulation->GetGripperForceCtrlOption();

    if(gripper_force_ctrl.gripper_force_ctrl_mission){
        mpc_manipulation->SetManipulationOption(gripper_force_ctrl);
    }

    else{

        gripper_force_ctrl.bend_deg = 113;

        gripper_force_ctrl.gripper_force_ctrl_mission = true;

        gripper_force_ctrl.forece_threshold = ui->ed_end_effector_load_threshold->text().toInt();

        mpc_manipulation->SetManipulationOption(gripper_force_ctrl);

        if(!mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_FORCE_CLRL)){
            QMessageBox::information(this, tr("Fail to Get pose"), tr("Check Gripper Status"));
            return;
        }
    }

}

void Manipulation_Dlg::SlotButtonHorizenDistance(){

    LRF_VEHICLE_HORIZEN_STRUCT lrf_vehicle;

    lrf_vehicle = mpc_manipulation->GetLRFVehicleHorizenOption();

    lrf_vehicle.inlier_distance = ui->ed_lrf_horizen_inlier->text().toDouble();
    lrf_vehicle.lrf_vehicle_mission = true;

    mpc_manipulation->SetManipulationOption(lrf_vehicle);

    if(!mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_VEHICLE_HORIZEN)){
        QMessageBox::information(this, tr("Fail to Get Horizen Dst"), tr("Check LRF Status"));
        return;
    }
}

//-------------------------------------------------
// View
//-------------------------------------------------
void Manipulation_Dlg::SlotViewCameraImage(cv::Mat _image){
    Display_Image(_image,mp_camera_image_grahicscene,ui->view_camera);
}

void Manipulation_Dlg::SlotViewSegnetImage(cv::Mat _image){
    Display_Image(_image,mp_segnet_image_grahicscene,ui->view_segnet);
}

void Manipulation_Dlg::SlotViewLRFImage(cv::Mat _image){

    Display_Image(_image,mp_lrf_image_grahicscene,ui->view_lrf);

}

QImage Manipulation_Dlg::Mat2QImage(cv::Mat src){
    cv::Mat temp; // make the same cv::Mat
    cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
}

void Manipulation_Dlg::Display_Image(cv::Mat _img, QGraphicsScene* _graphics_scene,QGraphicsView * _graphics_view,bool _fl_clear){

    if(_fl_clear){
        _graphics_scene->clear();
        _graphics_view->viewport()->update();
        return;
    }
    if(_img.rows <= 1 || _img.cols <= 1)
        return;

    cv::resize(_img,_img,cv::Size(),0.5,0.5);

    QImage tmp_qimg;
    QPixmap tmp_qpix;

    tmp_qimg = this->Mat2QImage(_img);
    tmp_qpix.convertFromImage(tmp_qimg);

    _graphics_scene->clear();
    //_graphics_scene = new QGraphicsScene(QRectF(0, 0, width, height), 0);
    _graphics_scene->addPixmap(tmp_qpix.scaled(QSize(
                        (int)_graphics_scene->width(), (int)_graphics_scene->height()),
                        Qt::KeepAspectRatio, Qt::SmoothTransformation));

    _graphics_view->fitInView(
                QRectF(0, 0, _graphics_view->geometry().width(), _graphics_view->geometry().height()),
                    Qt::KeepAspectRatio);

    _graphics_view->setScene(_graphics_scene);
    _graphics_view->show();
}
