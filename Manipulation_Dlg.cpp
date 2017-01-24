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

    int camera_view_width  = ui->view_lrf->geometry().width();
    int camera_view_height = ui->view_lrf->geometry().height();

    mp_camera_image_grahicscene = new QGraphicsScene(QRectF(0, 0, camera_view_width, camera_view_height), 0);


    //Button Connetion
    connect(ui->bt_kinova_init, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaInit()));

    connect(ui->bt_kinova_init_motion, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaInitMotion()));
    connect(ui->bt_kinova_align, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaAlignPanel()));
    connect(ui->bt_kinova_manipulate, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaDoManipulate()));

    connect(ui->bt_kinova_up, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepUp()));
    connect(ui->bt_kinova_down, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepDw()));
    connect(ui->bt_kinova_right, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepRi()));
    connect(ui->bt_kinova_left, SIGNAL(clicked()), this, SLOT(SlotButtonKinovaMoveStepLe()));

    connect(ui->bt_lrf_on, SIGNAL(clicked()), this, SLOT(SlotButtonLRFOn()));

    connect(ui->bt_camera_on, SIGNAL(clicked()), this, SLOT(SlotButtonCameraOn()));

    connect(mpc_manipulation, SIGNAL(SignalKinovaPosition(CartesianPosition)), this, SLOT(SlotEditeKinovaPosition(CartesianPosition)));

    //View Connetion
    connect(mpc_manipulation, SIGNAL(SignalLRFImage(cv::Mat)), this, SLOT(SlotViewLRFImage(cv::Mat)));
    connect(mpc_manipulation, SIGNAL(SignalCameraImage(cv::Mat)), this, SLOT(SlotViewCameraImage(cv::Mat)));

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

    position.Coordinates.ThetaX = ui->ed_kinova_roll->text().toFloat();
    position.Coordinates.ThetaY = ui->ed_kinova_pitch->text().toFloat();
    position.Coordinates.ThetaZ = ui->ed_kinova_yaw->text().toFloat();

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
        QMessageBox::information(this, tr("Fail to Align Panel"), tr("Check Kinova Status"));
}

void Manipulation_Dlg::SlotEditeKinovaPosition(CartesianPosition _position){

    ui->ed_kinova_x->setText(QString::number(_position.Coordinates.X, 'f', 4));
    ui->ed_kinova_y->setText(QString::number(_position.Coordinates.Y, 'f', 4));
    ui->ed_kinova_z->setText(QString::number(_position.Coordinates.Z, 'f', 4));

    ui->ed_kinova_roll->setText(QString::number(_position.Coordinates.ThetaX, 'f', 4));
    ui->ed_kinova_pitch->setText(QString::number(_position.Coordinates.ThetaY, 'f', 4));
    ui->ed_kinova_yaw->setText(QString::number(_position.Coordinates.ThetaZ, 'f', 4));

    return;
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

// Camera


// End Effector


//-------------------------------------------------
// View
//-------------------------------------------------
void Manipulation_Dlg::SlotViewCameraImage(cv::Mat _image){
    Display_Image(_image,mp_camera_image_grahicscene,ui->view_camera);
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
