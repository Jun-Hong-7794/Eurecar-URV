#include "Driving_Dlg.h"
#include "ui_Driving_Dlg.h"

Driving_Dlg::Driving_Dlg(CDriving* _pc_driving,QWidget *parent):
    QDialog(parent),
    ui(new Ui::Driving_Dlg)
{
    VTK_MODULE_INIT(vtkRenderingOpenGL2);
    ui->setupUi(this);

    qRegisterMetaType<QString>("QString");
    qRegisterMetaType<cv::Mat>("cv::Mat");

    //-------------------------------------------------
    // Graphic Scene Initialize
    //-------------------------------------------------
    int lrf_view_width  = ui->view_driving_lrf->geometry().width();
    int lrf_view_height = ui->view_driving_lrf->geometry().height();

    mp_lrf_image_grahicscene = new QGraphicsScene(QRectF(0, 0, lrf_view_width, lrf_view_height), 0);

    //-------------------------------------------------
    // Connection
    //-------------------------------------------------

    //Button Connect
    connect(ui->bt_vehicle_run,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleOperate()));
    connect(ui->bt_vehicle_connection,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleConnet()));
    connect(ui->bt_vehicle_set_option,SIGNAL(clicked()),this,SLOT(SlotButtonVehicleOptionSetting()));

    connect(ui->bt_velodyne_connection,SIGNAL(clicked()),this,SLOT(SlotButtonVelodyneConnet()));

    connect(ui->bt_mission_parking,SIGNAL(clicked()),this,SLOT(SlotButtonParking()));

    //Driving Class Init
    mpc_drivig = _pc_driving;

//    //PCL Init
//    mpc_drivig->PCLInit();
//    ui->qvtk_velodyne_driving_dlg->SetRenderWindow(mpc_drivig->GetPCL()->viewer->getRenderWindow());
//    (mpc_drivig->GetPCL())->viewer->setupInteractor(ui->qvtk_velodyne_driving_dlg->GetInteractor(),ui->qvtk_velodyne_driving_dlg->GetRenderWindow());
//    (mpc_drivig->GetPCL())->viewer->setBackgroundColor(0,0,0);
//    (mpc_drivig->GetPCL())->viewer->addCoordinateSystem(1.0);
//    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->cloud);
//    (mpc_drivig->GetPCL())->viewer->addPointCloud( (mpc_drivig->GetPCL())->waypoint_cloud);
//    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,5,"cloud");
//    (mpc_drivig->GetPCL())->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,10,"waypoint_cloud");

    // Velodyne View Connect
    connect(mpc_drivig,SIGNAL(SignalVelodyneParser(bool)),this,SLOT(SlotVeloyneParser(bool)));
    // LRF View Connect
    connect(mpc_drivig,SIGNAL(SignalLRFMapImage(cv::Mat)),this,SLOT(SlotLRFMapImage(cv::Mat)));

    //Edit Update
    connect(mpc_drivig,SIGNAL(SignalLRFVehicleAngleStruct(LRF_VEHICLE_ANGLE_STRUCT)),
            this,SLOT(SlotLRFVehicleAngleStructUpdate(LRF_VEHICLE_ANGLE_STRUCT)));
    connect(mpc_drivig,SIGNAL(SignalLRFVehicleHorizenStruct(LRF_VEHICLE_HORIZEN_STRUCT)),
            this,SLOT(SlotLRFVehicleHorizenStructUpdate(LRF_VEHICLE_HORIZEN_STRUCT)));


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

//    // Velodyne View Connect
//    connect(mpc_drivig,SIGNAL(SignalVelodyneParser(bool)),this,SLOT(SlotVeloyneParser(bool)));

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

void Driving_Dlg::SlotButtonParking(){
    if(!mpc_drivig->isRunning()){
//        if(!mpc_drivig->ConnectVehicle())
//            QMessageBox::information(this, tr("Fail to Connect Vehicle"), tr("Check Vehicle Status"));

        DRIVING_STRUCT driving_struct;

        driving_struct.velocity = ui->ed_vehicle_velocity->text().toInt();

        driving_struct.driving_mission = true;

        mpc_drivig->SetDrivingOption(driving_struct);

        mpc_drivig->SelectMainFunction(DRIVE_INX_PARKING__PANEL);

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

void Driving_Dlg::SlotVeloyneParser(bool _parser_complete){
    if(_parser_complete){

        ui->qvtk_velodyne_driving_dlg->update();
    }
}

void Driving_Dlg::SlotLRFMapImage(cv::Mat _image){

    Display_Image(_image,mp_lrf_image_grahicscene,ui->view_driving_lrf);
}

void Driving_Dlg::on_rd_vehicle_dir_forward_clicked()
{

}

QImage Driving_Dlg::Mat2QImage(cv::Mat src){

    cv::Mat temp; // make the same cv::Mat
    cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
}

void Driving_Dlg::Display_Image(cv::Mat _img, QGraphicsScene* _graphics_scene,QGraphicsView* _graphics_view,bool _fl_clear){

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

//-------------------------------------------------
// Edit View
//-------------------------------------------------

void Driving_Dlg::SlotLRFVehicleAngleStructUpdate(LRF_VEHICLE_ANGLE_STRUCT _lrf_struct){

    ui->ed_driving_lrf_v_distance->setText(QString::number(_lrf_struct.vertical_distance, 'f', 2));
    ui->ed_driving_lrf_slope->setText(QString::number(_lrf_struct.angle, 'f', 2));

}

void Driving_Dlg::SlotLRFVehicleHorizenStructUpdate(LRF_VEHICLE_HORIZEN_STRUCT _lrf_struct){

    ui->ed_driving_lrf_h_distance->setText(QString::number(_lrf_struct.horizen_distance, 'f', 2));

    ui->ed_driving_lrf_s_deg->setText(QString::number(_lrf_struct.s_inlier_deg, 'f', 2));
    ui->ed_driving_lrf_e_deg->setText(QString::number(_lrf_struct.e_inlier_deg, 'f', 2));

    double avr_deg = (_lrf_struct.s_inlier_deg + _lrf_struct.e_inlier_deg) / 2;
    ui->ed_driving_lrf_avr_deg->setText(QString::number(avr_deg, 'f', 2));

    ui->ed_driving_lrf_s_vir_deg->setText(QString::number(_lrf_struct.s_virtual_deg, 'f', 2));
    ui->ed_driving_lrf_e_vir_deg->setText(QString::number(_lrf_struct.e_virtual_deg, 'f', 2));

    double avr_vir_deg = (_lrf_struct.s_virtual_deg + _lrf_struct.e_virtual_deg) / 2;
    ui->ed_driving_lrf_avr_vir_deg->setText(QString::number(avr_vir_deg, 'f', 2));
}










