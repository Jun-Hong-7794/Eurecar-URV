#include "Manipulation_Dlg.h"
#include "ui_Manipulation_Dlg.h"

Manipulation_Dlg::Manipulation_Dlg(CManipulation* _pc_manipulation, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Manipulation_Dlg){

    ui->setupUi(this);

    mpc_manipulation = _pc_manipulation;

    m_valve_size_graph_num = -1;
    m_valve_anal_graph_num = -1;

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

    int valve_view_width  = ui->view_valve_modeling->geometry().width();
    int valve_view_height = ui->view_valve_modeling->geometry().height();

    mp_valve_image_grahicscene = new QGraphicsScene(QRectF(0, 0, valve_view_width, valve_view_height), 0);

    int panel_view_width  = ui->view_panel_modeling->geometry().width();
    int panel_view_height = ui->view_panel_modeling->geometry().height();

    mp_panel_image_grahicscene = new QGraphicsScene(QRectF(0, 0, panel_view_width, panel_view_height), 0);

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

    connect(ui->bt_kinova_base_rotate, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaBaseRotate()));

    connect(ui->bt_kinova_get_force, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaGetForceFeedBackData()));
    connect(ui->bt_kinova_set_force_thresh, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaSetForceThreshData()));

    connect(ui->bt_lrf_kinova_ctrl, SIGNAL(clicked()), this, SLOT(SlotButtonLRFKinovaCtrl()));
    connect(ui->bt_kinova_force_ctrl, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaForceCtrl()));
    connect(ui->bt_gripper_force_ctrl, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorLoadCheckIter()));
    connect(ui->bt_lrf_horizent_distance, SIGNAL(clicked()), this, SLOT(SlotButtonHorizenDistance()));


    connect(ui->bt_end_effector_grasp_2, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorGrasp()));
    connect(ui->bt_end_effector_present_pos_check, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorPoseCheck()));
    connect(ui->bt_end_effector_org_pos, SIGNAL(clicked()), this, SLOT(SlotButtonEEffectorGoToOrg()));

    connect(ui->bt_lrf_on, SIGNAL(clicked()), this, SLOT(SlotButtonLRFOn()));
    connect(ui->bt_camera_on, SIGNAL(clicked()), this, SLOT(SlotButtonCameraOn()));

    connect(ui->bt_lrf_get_info, SIGNAL(clicked()), this, SLOT(SlotButtonGetLRFInfo()));
    connect(ui->bt_kinova_get_pose, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaGetPosition()));

    connect(ui->bt_gripper_grasp, SIGNAL(clicked()), this, SLOT(SlotButtonGripperGrasp()));

    connect(ui->bt_graph_clear, SIGNAL(clicked()), this, SLOT(SlotButtonGraphClear()));
    connect(ui->bt_graph_load, SIGNAL(clicked()), this, SLOT(SlotButtonLoadGraphData()));
    connect(ui->bt_graph_save, SIGNAL(clicked()), this, SLOT(SlotButtonSaveGraphData()));
    connect(ui->bt_graph_dat_analisys, SIGNAL(clicked()), this, SLOT(SlotButtonAnalisysGraphData()));

    connect(ui->bt_anal_graph_analisys, SIGNAL(clicked()), this, SLOT(SlotButtonAnalDataAnalisys()));
    connect(ui->bt_anal_graph_load, SIGNAL(clicked()), this, SLOT(SlotButtonAnalLoadGraphData()));
    connect(ui->bt_anal_graph_clear, SIGNAL(clicked()), this, SLOT(SlotButtonAnalGraphClear()));

    connect(mpc_manipulation, SIGNAL(SignalKinovaPosition(CartesianPosition)), this, SLOT(SlotEditeKinovaPosition(CartesianPosition)));
    connect(mpc_manipulation, SIGNAL(SignalKinovaForceVector(CartesianPosition)), this, SLOT(SlotEditeKinovaForceVector(CartesianPosition)));
    connect(mpc_manipulation, SIGNAL(SignalKinovaForceVector(CartesianPosition)), this, SLOT(SlotKinovaForceVectorData(CartesianPosition)));

    connect(mpc_manipulation, SIGNAL(SignalKinovaForceCheckOption(KINOVA_FORCE_CHECK_STRUCT)), this, SLOT(SlotSetForceGraphThresh(KINOVA_FORCE_CHECK_STRUCT)));

    connect(mpc_manipulation, SIGNAL(SignalLRFHorizentDistance(LRF_VEHICLE_HORIZEN_STRUCT)), this, SLOT(SlotLRFHorizentDistance(LRF_VEHICLE_HORIZEN_STRUCT)));

    //Edit Update
    connect(mpc_manipulation, SIGNAL(SignalLRFKinovaAngleStruct(LRF_KINOVA_ANGLE_CTRL_STRUCT)),
            this, SLOT(SlotEditeLRFKinovaAngleStruct(LRF_KINOVA_ANGLE_CTRL_STRUCT)));
    connect(mpc_manipulation, SIGNAL(SignalLRFKinovaHorizenStruct(LRF_KINOVA_HORIZEN_CTRL_STRUCT)),
            this, SLOT(SlotEditeLRFKinovaHorizenStruct(LRF_KINOVA_HORIZEN_CTRL_STRUCT)));
    connect(mpc_manipulation, SIGNAL(SignalLRFKinovaVerticalStruct(LRF_KINOVA_VERTICAL_CTRL_STRUCT)),
            this, SLOT(SlotEditeLRFKinovaVertivalStruct(LRF_KINOVA_VERTICAL_CTRL_STRUCT)));

    connect(mpc_manipulation, SIGNAL(SignalEditeGripperStatus(GRIPPER_STATUS)), this, SLOT(SlotEditeGripperStatus(GRIPPER_STATUS)));

    //Check Button
    connect(ui->ck_segnet_switch, SIGNAL(clicked(bool)), this, SLOT(SlotButtonSegnetOn(bool)));

    //View Connetion
    connect(mpc_manipulation, SIGNAL(SignalLRFImage(cv::Mat)), this, SLOT(SlotViewLRFImage(cv::Mat)));
    connect(mpc_manipulation, SIGNAL(SignalCameraImage(cv::Mat)), this, SLOT(SlotViewCameraImage(cv::Mat)));
    connect(mpc_manipulation, SIGNAL(SignalSegnetImage(cv::Mat)), this, SLOT(SlotViewSegnetImage(cv::Mat)));
    connect(mpc_manipulation, SIGNAL(SignalValveImage(cv::Mat)), this, SLOT(SlotValveImage(cv::Mat)));
    connect(mpc_manipulation, SIGNAL(SignalPanelImage(cv::Mat)), this, SLOT(SlotPanelImage(cv::Mat)));

    connect(mpc_manipulation, SIGNAL(SignalValveSizeData(QVector<double>,QVector<double>,int)),
            this, SLOT(SlotValveSizeData(QVector<double>,QVector<double>,int)));

    QVector<double> x, y;

    for(int i = 0; i < 36; i++){
        double data_y;

        double a = (220 - 190) / 2;
        double d = (220 + 190) / 2;

        data_y = a * sin(( (2*KINOVA_PI) / 36)*i) + d;
        x.push_back((double)i);
        y.push_back(data_y);
    }

    int graph_index = GetValveSizeDataGraphCurrentIndex() + 1;
    mtx_valve_data_graph.lock();
    {
        SetValveSizeData(ui->graph_gripper_data_plot, x, y,
                         mqvec_valve_data_x, mqvec_valve_data_y,
                         m_valve_size_graph_num, graph_index, "Test Data");
    }
    mtx_valve_data_graph.unlock();

    //-------------------------------------------
    // Graph_gripper_data_plot
    //-------------------------------------------
    ui->graph_gripper_data_plot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    ui->graph_gripper_data_plot->legend->setVisible(true);
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    ui->graph_gripper_data_plot->legend->setFont(legendFont);
    ui->graph_gripper_data_plot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    ui->graph_gripper_data_plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    // give the axes some labels:
    ui->graph_gripper_data_plot->xAxis->setLabel("Trial");
    ui->graph_gripper_data_plot->yAxis->setLabel("Diff Step");
    // set axes ranges, so we see all data:
    ui->graph_gripper_data_plot->xAxis->setRange(0, 36);
    ui->graph_gripper_data_plot->yAxis->setRange(150, 450);
    ui->graph_gripper_data_plot->replot();

    //-------------------------------------------
    // Graph_gripper_data_anal_plot
    //-------------------------------------------
    ui->graph_gripper_data_anal_plot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    ui->graph_gripper_data_anal_plot->legend->setVisible(true);
    ui->graph_gripper_data_anal_plot->legend->setFont(legendFont);
    ui->graph_gripper_data_anal_plot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    ui->graph_gripper_data_anal_plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    // give the axes some labels:
    ui->graph_gripper_data_anal_plot->xAxis->setLabel("Trial");
    ui->graph_gripper_data_anal_plot->yAxis->setLabel("Diff Step");
    // set axes ranges, so we see all data:
    ui->graph_gripper_data_anal_plot->xAxis->setRange(0, 36);
    ui->graph_gripper_data_anal_plot->yAxis->setRange(150, 450);
    ui->graph_gripper_data_anal_plot->replot();

    //-------------------------------------------
    // Graph_kinova_force_plot_x
    //-------------------------------------------
    ui->graph_kinova_force_plot_x->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    ui->graph_kinova_force_plot_x->legend->setVisible(true);
    legendFont.setPointSize(9); // and make a bit smaller for legend
    ui->graph_kinova_force_plot_x->legend->setFont(legendFont);
    ui->graph_kinova_force_plot_x->legend->setBrush(QBrush(QColor(255,255,255,230)));
    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    ui->graph_kinova_force_plot_x->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    // give the axes some labels:
    ui->graph_kinova_force_plot_x->yAxis->setLabel("Force(N)");
    // set axes ranges, so we see all data:
    ui->graph_kinova_force_plot_x->xAxis->setRange(0, 30);
    ui->graph_kinova_force_plot_x->yAxis->setRange(-15, 15);
    ui->graph_kinova_force_plot_x->replot();

    ui->graph_kinova_force_plot_x->addGraph();//For Data
    ui->graph_kinova_force_plot_x->addGraph();//For Thresh

    //-------------------------------------------
    // Graph_kinova_force_plot_y
    //-------------------------------------------
    ui->graph_kinova_force_plot_y->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    ui->graph_kinova_force_plot_y->legend->setVisible(true);
    legendFont.setPointSize(9); // and make a bit smaller for legend
    ui->graph_kinova_force_plot_y->legend->setFont(legendFont);
    ui->graph_kinova_force_plot_y->legend->setBrush(QBrush(QColor(255,255,255,230)));
    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    ui->graph_kinova_force_plot_y->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    // give the axes some labels:
    ui->graph_kinova_force_plot_y->yAxis->setLabel("Force(N)");
    // set axes ranges, so we see all data:
    ui->graph_kinova_force_plot_y->xAxis->setRange(0, 30);
    ui->graph_kinova_force_plot_y->yAxis->setRange(-15, 15);
    ui->graph_kinova_force_plot_y->replot();

    ui->graph_kinova_force_plot_y->addGraph();//For Data
    ui->graph_kinova_force_plot_y->addGraph();//For Thresh

    //-------------------------------------------
    // Graph_kinova_force_plot_z
    //-------------------------------------------
    ui->graph_kinova_force_plot_z->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    ui->graph_kinova_force_plot_z->legend->setVisible(true);
    legendFont.setPointSize(9); // and make a bit smaller for legend
    ui->graph_kinova_force_plot_z->legend->setFont(legendFont);
    ui->graph_kinova_force_plot_z->legend->setBrush(QBrush(QColor(255,255,255,230)));
    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    ui->graph_kinova_force_plot_z->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    // give the axes some labels:
    ui->graph_kinova_force_plot_z->yAxis->setLabel("Force(N)");
    // set axes ranges, so we see all data:
    ui->graph_kinova_force_plot_z->xAxis->setRange(0, 30);
    ui->graph_kinova_force_plot_z->yAxis->setRange(-15, 15);
    ui->graph_kinova_force_plot_z->replot();

    ui->graph_kinova_force_plot_z->addGraph();//For Data
    ui->graph_kinova_force_plot_z->addGraph();//For Thresh

    for(int i =0; i < 30; i++){
        mqvec_kinova_force_thresh_x.push_back(0.0);
        mqvec_kinova_force_thresh_y.push_back(0.0);
        mqvec_kinova_force_thresh_z.push_back(0.0);
        mqvec_kinova_force_thresh_time.push_back(i);
    }

    QString fileName = "/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/ValveSizeData/ModelData/";

    QVector<double> x_16, y_16;
    QVector<double> x_17, y_17;
    QVector<double> x_18, y_18;
    QVector<double> x_19, y_19;
    QVector<double> x_22, y_22;
    QVector<double> x_24, y_24;

    InterpreteValveSizeDataFile(fileName + "16mm.txt", x_16, y_16);
    InterpreteValveSizeDataFile(fileName + "17mm.txt", x_17, y_17);
    InterpreteValveSizeDataFile(fileName + "18mm.txt", x_18, y_18);
    InterpreteValveSizeDataFile(fileName + "19mm.txt", x_19, y_19);
    InterpreteValveSizeDataFile(fileName + "22mm.txt", x_22, y_22);
    InterpreteValveSizeDataFile(fileName + "24mm.txt", x_24, y_24);

    mpc_manipulation->ValveSizeDataModelInit(y_16, 16);
    mpc_manipulation->ValveSizeDataModelInit(y_17, 17);
    mpc_manipulation->ValveSizeDataModelInit(y_18, 18);
    mpc_manipulation->ValveSizeDataModelInit(y_19, 19);
    mpc_manipulation->ValveSizeDataModelInit(y_22, 22);
    mpc_manipulation->ValveSizeDataModelInit(y_24, 24);

    mpc_manipulation->ValveModeling(19,30);
    mpc_manipulation->PanelModeling(19,280,380,-2);
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

void Manipulation_Dlg::SlotButtonKinovaBaseRotate(){

    double rotate_value = ui->ed_kinova_base_rotate->text().toDouble();

    if(!mpc_manipulation->KinovaRotateBase(rotate_value))
        QMessageBox::information(this, tr("Fail to Base Rotate"), tr("Check Kinova Status"));;
}

 void Manipulation_Dlg::SlotButtonKinovaGetForceFeedBackData(){

     KINOVA_FORCE_CHECK_STRUCT force_check;
     if(mpc_manipulation->isRunning()){
         force_check.fl_kinova_force_sensing_option = false;
         mpc_manipulation->SetManipulationOption(force_check);

         ui->bt_kinova_get_force->setText("Get Force");
     }
     else{
         force_check.fl_kinova_force_sensing_option = true;
         mpc_manipulation->SetManipulationOption(force_check);
         mpc_manipulation->SelectMainFunction(MANIPUL_INX_KINOVA_FORCE_CHECK);
         ui->bt_kinova_get_force->setText("Stop");
     }
}

 void Manipulation_Dlg::SlotButtonKinovaSetForceThreshData(){

     int current_graph = 0;

     QString force_x;
     QString force_y;
     QString force_z;

     force_x = ui->ed_kinova_force_thresh_x->text();
     force_y = ui->ed_kinova_force_thresh_y->text();
     force_z = ui->ed_kinova_force_thresh_z->text();

     for(int i = 0; i < 30;i++){
        mqvec_kinova_force_thresh_x[i] = force_x.toDouble();
        mqvec_kinova_force_thresh_y[i] = force_y.toDouble();
        mqvec_kinova_force_thresh_z[i] = force_z.toDouble();
     }

     mtx_kinova_force_vector_graph.lock();
     {
         SetValveSizeData(ui->graph_kinova_force_plot_x, mqvec_kinova_force_thresh_time,
                          mqvec_kinova_force_thresh_x, mqvec_kinova_force_x_data_x, mqvec_kinova_force_x_data_y, current_graph, 0, "X_Thresh");
         SetValveSizeData(ui->graph_kinova_force_plot_y, mqvec_kinova_force_thresh_time,
                          mqvec_kinova_force_thresh_y, mqvec_kinova_force_y_data_x, mqvec_kinova_force_y_data_y, current_graph, 0, "Y_Thresh");
         SetValveSizeData(ui->graph_kinova_force_plot_z, mqvec_kinova_force_thresh_time,
                          mqvec_kinova_force_thresh_z, mqvec_kinova_force_z_data_x, mqvec_kinova_force_z_data_y, current_graph, 0, "Z_Thresh");
     }
     mtx_kinova_force_vector_graph.unlock();

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

void Manipulation_Dlg::SlotSetForceGraphThresh(KINOVA_FORCE_CHECK_STRUCT _kinova_force_option){

    ui->ed_kinova_force_thresh_x->setText(QString::number(_kinova_force_option.force_threshold_x));
    ui->ed_kinova_force_thresh_y->setText(QString::number(_kinova_force_option.force_threshold_y));
    ui->ed_kinova_force_thresh_z->setText(QString::number(_kinova_force_option.force_threshold_z));

    SlotButtonKinovaSetForceThreshData();
}

void Manipulation_Dlg::SlotEditeLRFKinovaAngleStruct(LRF_KINOVA_ANGLE_CTRL_STRUCT _lrf_kinova_option){

    ui->ed_lrf_slope->setText(QString::number(_lrf_kinova_option.slope));
    ui->ed_lrf_distance->setText(QString::number(_lrf_kinova_option.current_distance));
}

void Manipulation_Dlg::SlotEditeLRFKinovaHorizenStruct(LRF_KINOVA_HORIZEN_CTRL_STRUCT _lrf_kinova_option){

    ui->ed_lrf_horizen_s_deg->setText(QString::number(_lrf_kinova_option.inlier_deg_s_output));
    ui->ed_lrf_horizen_e_deg->setText(QString::number(_lrf_kinova_option.inlier_deg_e_output));

    double deg_avr = (_lrf_kinova_option.inlier_deg_s_output + _lrf_kinova_option.inlier_deg_e_output) / 2;
    ui->ed_lrf_horizen_deg_avr->setText(QString::number(deg_avr));

    ui->ed_lrf_horizen_distance->setText(QString::number(_lrf_kinova_option.current_h_distance));
    ui->ed_lrf_horizen_inlier->setText(QString::number(_lrf_kinova_option.inlier_lrf_dst));


}

void Manipulation_Dlg::SlotEditeLRFKinovaVertivalStruct(LRF_KINOVA_VERTICAL_CTRL_STRUCT _lrf_kinova_option){

    ui->ed_lrf_slope->setText(QString::number(_lrf_kinova_option.slope));
    ui->ed_lrf_distance->setText(QString::number(_lrf_kinova_option.current_distance));

}

void Manipulation_Dlg::SlotEditeGripperStatus(GRIPPER_STATUS _gripper_status){

    ui->ed_gripper_1_pose->setText(QString::number(_gripper_status.present_pose_1));
    ui->ed_gripper_2_pose->setText(QString::number(_gripper_status.present_pose_2));

    double diff_1 = _gripper_status.present_pose_1 - 1700;
    double diff_2 = _gripper_status.present_pose_2 - 1700;

    double diff_pose = fabs(diff_1 + diff_2);
    ui->ed_gripper_diff_pose->setText(QString::number(diff_pose));

    ui->ed_gripper_1_current_load->setText(QString::number(_gripper_status.present_load_1));
    ui->ed_gripper_2_current_load->setText(QString::number(_gripper_status.present_load_2));
}

void Manipulation_Dlg::SlotButtonKinovaForceCtrl(){

    KINOVA_FORCE_CTRL_STRUCT kinova_force_ctrl_option;

    kinova_force_ctrl_option.kinova_force_ctrl_mission = true;
    kinova_force_ctrl_option.step_count = ui->ed_kinova_force_step_count->text().toDouble();
    kinova_force_ctrl_option.force_threshold = ui->ed_kinova_force_threshold->text().toDouble();

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

    LRF_KINOVA_VERTICAL_CTRL_STRUCT lrf_kinova_option;

    lrf_kinova_option.desired_distance = ui->ed_lrf_kinova_desired_dst->text().toDouble();
    lrf_kinova_option.error = ui->ed_lrf_kinova_error->text().toDouble();
    lrf_kinova_option.s_deg = ui->ed_lrf_kinova_s_deg->text().toDouble();
    lrf_kinova_option.e_deg = ui->ed_lrf_kinova_e_deg->text().toDouble();

    mpc_manipulation->SetManipulationOption(lrf_kinova_option);

    if(!mpc_manipulation->SelectMainFunction(MANIPUL_INX_LRF_KINOVA_VERTIVAL_CTRL))
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

void Manipulation_Dlg::SlotButtonHorizenDistance(){

    double current_h_distance = 0;

    double inlier_s_deg = 0;
    double inlier_e_deg = 0;
    double s_virture_deg = 0;
    double e_virture_deg = 0;

    int sampling_loop = 1;

    double inlier_lrf_dst = ui->ed_lrf_horizen_inlier->text().toDouble();

    double s_deg = ui->ed_lrf_horizen_input_s_deg->text().toDouble();
    double e_deg = ui->ed_lrf_horizen_input_e_deg->text().toDouble();

    if(!mpc_manipulation->GetHorizenDistance(inlier_lrf_dst, current_h_distance, inlier_s_deg, inlier_e_deg,
                                  s_virture_deg, e_virture_deg, s_deg, e_deg, sampling_loop)){
        QMessageBox::information(this, tr("Fail to Get LRF Info"), tr("Check LRF Status"));
        return;
    }
    else{
        ui->ed_lrf_horizen_s_deg->setText(QString::number(inlier_s_deg, 'f', 3));
        ui->ed_lrf_horizen_e_deg->setText(QString::number(inlier_e_deg, 'f', 3));
        ui->ed_lrf_horizen_distance->setText(QString::number(current_h_distance, 'f', 3));

        double avr = (inlier_s_deg + inlier_e_deg) / 2;
        ui->ed_lrf_horizen_deg_avr->setText(QString::number(avr, 'f', 3));
    }
}

// End Effector
void Manipulation_Dlg::SlotButtonEEffectorGrasp(){


}

void Manipulation_Dlg::SlotButtonEEffectorPoseCheck(){

//    uint16_t present_pos = 0;

//    if(!mpc_manipulation->GripperPresentPose(present_pos)){
//        QMessageBox::information(this, tr("Fail to Get pose"), tr("Check Gripper Status"));
//        return;
//    }

//    ui->ed_end_effector_present_pos->setText(QString::number(present_pos));
}

void Manipulation_Dlg::SlotButtonEEffectorGoToOrg(){

//    uint16_t org_pos = 13;

//    if(!mpc_manipulation->GripperGoThePose(org_pos)){
//        QMessageBox::information(this, tr("Fail to Get pose"), tr("Check Gripper Status"));
//        return;
//    }

}

void Manipulation_Dlg::SlotButtonEEffectorLoadCheckIter(){

//    GRIPPER_FORCE_CTRL_STRUCT gripper_force_ctrl;

//    gripper_force_ctrl = mpc_manipulation->GetGripperForceCtrlOption();

//    if(gripper_force_ctrl.gripper_force_ctrl_mission){
//        mpc_manipulation->SetManipulationOption(gripper_force_ctrl);
//    }

//    else{

//        gripper_force_ctrl.bend_deg = 113;

//        gripper_force_ctrl.gripper_force_ctrl_mission = true;

//        gripper_force_ctrl.force_threshold = ui->ed_end_effector_load_threshold->text().toInt();

//        mpc_manipulation->SetManipulationOption(gripper_force_ctrl);

//        if(!mpc_manipulation->SelectMainFunction(MANIPUL_INX_GRIPPER_FORCE_CLRL)){
//            QMessageBox::information(this, tr("Fail to Get pose"), tr("Check Gripper Status"));
//            return;
//        }
//    }
}

void Manipulation_Dlg::SlotButtonGripperGrasp(){

    int pose_1 = ui->ed_gripper_1_goal_pos->text().toInt();
    int pose_2 = ui->ed_gripper_2_goal_pos->text().toInt();

    int load_thresh = ui->ed_gripper_1_load_thresh->text().toInt();

    if(!mpc_manipulation->GripperGoThePose(pose_1, pose_2, load_thresh))
        QMessageBox::information(this, tr("Fail to Go to the pose"), tr("Check Gripper Status"));

}

void Manipulation_Dlg::SlotButtonGripperTorqueOn(){

}

void Manipulation_Dlg::SlotButtonGraphClear(){

    mqvec_valve_data_x.clear();
    mqvec_valve_data_y.clear();

    ui->graph_gripper_data_plot->clearGraphs();
    ui->graph_gripper_data_plot->replot();
    m_valve_size_graph_num = -1;
}

void Manipulation_Dlg::SlotButtonSaveGraphData(){

    if(m_valve_size_graph_num < 0){
        QMessageBox::information(this, tr("Fail to Save Data"), tr("There is no data on graph!!"));
        return;
    }

    int graph_index = ui->ed_graph_data_save_index->text().toInt();

    if(graph_index > m_valve_size_graph_num){
        QMessageBox::information(this, tr("Fail to Save Data"), tr("There is no data on graph!!"));
        return;
    }

    QFileDialog dialog(this);
    QString fileName = dialog.getSaveFileName(this,
         tr("Save Data"), "/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/ValveSizeData/",
                                              tr("Text File (*.txt)"));

    if(fileName.isEmpty())
        return;

    if(!fileName.contains(".txt"))
        fileName += ".txt";

    QFile file(fileName);

    if (file.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file);

        for(int j = 0; j < mqvec_valve_data_x.at(graph_index).size(); j++){

            QString str_x_value = QString::number(mqvec_valve_data_x.at(graph_index).at(j));
            QString str_y_value = QString::number(mqvec_valve_data_y.at(graph_index).at(j));

            stream << "(" << str_x_value << "," << str_y_value << ")" << endl;
        }
    }
}

void Manipulation_Dlg::SlotButtonLoadGraphData(){

    QFileDialog dialog(this);
    QStringList fileName;

    dialog.setDirectory("/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/ValveSizeData/");

    dialog.setFileMode(QFileDialog::AnyFile);

    if(dialog.exec())
        fileName = dialog.selectedFiles();
    else
        return;

    QFile inputFile(fileName[0]);

    if (inputFile.open(QIODevice::ReadOnly))
    {
       QVector<double> _x;
       QVector<double> _y;
       QString _title = "Empty";

       QTextStream in(&inputFile);

       while (!in.atEnd())
       {
           InterpreteValveSizeDataLine(in.readLine(), _x, _y, _title);
       }
       inputFile.close();

       int graph_index = GetValveSizeDataGraphCurrentIndex() + 1;
       mtx_valve_data_graph.lock();
       {
            SetValveSizeData(ui->graph_gripper_data_plot,_x, _y,
                             mqvec_valve_data_x, mqvec_valve_data_y,
                             m_valve_size_graph_num, graph_index, _title);
       }
       mtx_valve_data_graph.unlock();
    }
    else{
        std::cout<< "Mission File Open Error!" << std::endl;
        return;
    }

}

void Manipulation_Dlg::SlotButtonAnalisysGraphData(){

}

void Manipulation_Dlg::SlotButtonAnalGraphClear(){

    mqvec_valve_anal_x.clear();
    mqvec_valve_anal_y.clear();

    ui->graph_gripper_data_anal_plot->clearGraphs();
    ui->graph_gripper_data_anal_plot->replot();

    m_valve_anal_graph_num = -1;
}

void Manipulation_Dlg::SlotButtonAnalLoadGraphData(){

    QFileDialog dialog(this);
    QStringList fileName;

    dialog.setDirectory("/home/winner/Workspace/2017MBZIRC_Code/Eurecar-URV/Eurecar-URV/ValveSizeData/");

    dialog.setFileMode(QFileDialog::AnyFile);

    if(dialog.exec())
        fileName = dialog.selectedFiles();
    else
        return;

    QFile inputFile(fileName[0]);

    if (inputFile.open(QIODevice::ReadOnly))
    {
       QVector<double> _x;
       QVector<double> _y;
       QString _title = "Empty";

       QTextStream in(&inputFile);

       while (!in.atEnd())
       {
           InterpreteValveSizeDataLine(in.readLine(), _x, _y, _title);
       }
       inputFile.close();

       int graph_index = GetValveSizeAnalGraphCurrentIndex() + 1;

       mtx_valve_anal_graph.lock();
       {
           QVector<double> sorted_y;

           sorted_y = mpc_manipulation->DataSort(_y);

           SetValveSizeData(ui->graph_gripper_data_anal_plot,_x, sorted_y,
                             mqvec_valve_anal_x, mqvec_valve_anal_y,
                             m_valve_anal_graph_num, graph_index, _title);
       }
       mtx_valve_anal_graph.unlock();
    }
    else{
        std::cout<< "Mission File Open Error!" << std::endl;
        return;
    }

}

void Manipulation_Dlg::SlotButtonAnalDataAnalisys(){

    if(m_valve_anal_graph_num < 0){
        QMessageBox::information(this, tr("Fail to Save Data"), tr("There is no data on graph!!"));
        return;
    }

    int graph_index = ui->ed_anal_graph_label->text().toInt();

    if(graph_index > m_valve_anal_graph_num){
        QMessageBox::information(this, tr("Fail to Save Data"), tr("There is no data on graph!!"));
        return;
    }

    int result = mpc_manipulation->DataAnalisys(mqvec_valve_anal_y.at(graph_index));

    if(result == 0){
        QMessageBox::information(this, tr("Fail to Analyse"), tr("Unclassifiable Data"));
        return;
    }

    QString str_valve_size;

    str_valve_size = QString::number(result);
    ui->txt_anal_data_info->append(str_valve_size + "mm");
}

//-------------------------------------------------
// Interprete
//-------------------------------------------------
void Manipulation_Dlg::InterpreteValveSizeDataFile(QString _file, QVector<double>& _x, QVector<double>& _y){

    QFile inputFile(_file);

    if (inputFile.open(QIODevice::ReadOnly))
    {
       QString _title = "Empty";

       QTextStream in(&inputFile);

       while (!in.atEnd())
       {
           InterpreteValveSizeDataLine(in.readLine(), _x, _y, _title);
       }
       inputFile.close();
    }
}

bool Manipulation_Dlg::InterpreteValveSizeDataLine(QString _line, QVector<double>& _x, QVector<double>& _y, QString& _title){

    if(_line.contains("Title")){

        int colone_index = _line.indexOf(":");

        _title = _line.mid(colone_index + 1).trimmed();

        return true;
    }
    else if(_line.contains("(")){
        int bracket_small_op_index = _line.indexOf("(");
        int bracket_small_ed_index = _line.indexOf(")");
        int comma_index = _line.indexOf(",");

        if((bracket_small_op_index == -1) || (bracket_small_ed_index == -1) || (comma_index == -1))
            return false;

        _x.push_back(_line.mid(bracket_small_op_index + 1, comma_index - bracket_small_op_index - 1).toDouble());
        _y.push_back(_line.mid(comma_index + 1, bracket_small_ed_index - comma_index - 1).toDouble());
    }

    return true;
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

void Manipulation_Dlg::SlotValveImage(cv::Mat _image){

    Display_Image(_image,mp_valve_image_grahicscene,ui->view_valve_modeling);
}

void Manipulation_Dlg::SlotPanelImage(cv::Mat _image){

    Display_Image(_image,mp_panel_image_grahicscene,ui->view_panel_modeling);
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

void Manipulation_Dlg::SlotValveSizeData(QVector<double> _x, QVector<double> _y, int _graph_index){

    int graph_index = GetValveSizeDataGraphCurrentIndex();

    //create graph and assign data to it:
    if(_graph_index == -1){
        graph_index += 1;
    }

    mtx_valve_data_graph.lock();
    {
        SetValveSizeData(ui->graph_gripper_data_plot, _x, _y, mqvec_valve_data_x, mqvec_valve_data_y, m_valve_size_graph_num, graph_index);
    }
    mtx_valve_data_graph.unlock();
}

void Manipulation_Dlg::SlotKinovaForceVectorData(CartesianPosition _force_vector){

    int current_graph = 0;

    if(mqvec_kinova_force_data_x.size() > 30){
        mqvec_kinova_force_data_x.pop_back();
        mqvec_kinova_force_data_y.pop_back();
        mqvec_kinova_force_data_z.pop_back();

        mqvec_kinova_force_data_time.pop_back();
    }

    int time = mqvec_kinova_force_data_time.size();

    mqvec_kinova_force_data_time.push_back(time);

    mqvec_kinova_force_data_x.push_front(_force_vector.Coordinates.X);
    mqvec_kinova_force_data_y.push_front(_force_vector.Coordinates.Y);
    mqvec_kinova_force_data_z.push_front(_force_vector.Coordinates.Z);

    ui->ed_kinova_force_current_x->setText(QString::number(_force_vector.Coordinates.X));
    ui->ed_kinova_force_current_y->setText(QString::number(_force_vector.Coordinates.Y));
    ui->ed_kinova_force_current_z->setText(QString::number(_force_vector.Coordinates.Z));

    mtx_kinova_force_vector_graph.lock();
    {
        SetValveSizeData(ui->graph_kinova_force_plot_x, mqvec_kinova_force_data_time,
                         mqvec_kinova_force_data_x, mqvec_kinova_force_x_data_x, mqvec_kinova_force_x_data_y, current_graph, 0, "Force_X");
        SetValveSizeData(ui->graph_kinova_force_plot_y, mqvec_kinova_force_data_time,
                         mqvec_kinova_force_data_y, mqvec_kinova_force_y_data_x, mqvec_kinova_force_y_data_y, current_graph, 0, "Force_Y");
        SetValveSizeData(ui->graph_kinova_force_plot_z, mqvec_kinova_force_data_time,
                         mqvec_kinova_force_data_z, mqvec_kinova_force_z_data_x, mqvec_kinova_force_z_data_y, current_graph, 0, "Force_Z");
    }
    mtx_kinova_force_vector_graph.unlock();
}

int Manipulation_Dlg::GetValveSizeDataGraphCurrentIndex(){

    return m_valve_size_graph_num;;
}

int Manipulation_Dlg::GetValveSizeAnalGraphCurrentIndex(){

    return m_valve_anal_graph_num;
}

void Manipulation_Dlg::SetValveSizeData(QCustomPlot* _plot, QVector<double> _x, QVector<double> _y,
                                        QVector<QVector<double>>& _contain_x,QVector<QVector<double>>& _contain_y,
                                        int& _current_index, int _graph_index, QString _data_name){

    if(_graph_index > _current_index){
        _plot->addGraph();
        _current_index++;
        _contain_x.push_back(_x);
        _contain_y.push_back(_y);
    }

//    _contain_x[_graph_index] = _x;
//    _contain_y[_graph_index] = _y;

    int color_index = _graph_index - (_graph_index / 12)*12 + 7;

    _plot->graph(_graph_index)->setPen(QPen(Qt::GlobalColor(color_index)));
    if(_data_name != "Empty"){
        _plot->graph(_graph_index)->setName("Graph" + QString::number(_graph_index) + ": " + _data_name);
    }
    else{
        _plot->graph(_graph_index)->setName("Graph" + QString::number(_graph_index) + ": New Data");
    }
//  ui->graph_gripper_data_plot->graph(_graph_index)->setBrush(QBrush(QColor(255, 0, 0, 20)));
    _plot->graph(_graph_index)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    _plot->graph(_graph_index)->setData(_x, _y);
    _plot->replot();
}



