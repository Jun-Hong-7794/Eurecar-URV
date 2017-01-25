#ifndef MANIPULATION_DLG_H
#define MANIPULATION_DLG_H

#include <QDialog>
//-------------------------------------------------
// Device Class
//-------------------------------------------------
#include "Device_Class/CGPS/CGPS.h"
#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"
#include "Device_Class/CKinova/CKinova.h"
#include "Device_Class/CVehicle/CVehicle.h"
#include "Device_Class/CVelodyne/CVelodyne.h"

//-------------------------------------------------
// Mission Class
//-------------------------------------------------
#include "Mission_Class/CManipulation/CManipulation.h"

namespace Ui {
class Manipulation_Dlg;
}

class Manipulation_Dlg : public QDialog
{
    Q_OBJECT

public:
    explicit Manipulation_Dlg(CManipulation* _pc_manipulation, QWidget *parent = 0);
    ~Manipulation_Dlg();

private:
    Ui::Manipulation_Dlg *ui;

private:
    //-------------------------------------------------
    // Mission Class
    //-------------------------------------------------
    CManipulation* mpc_manipulation;

private:// Graphic Scene
    QGraphicsScene *mp_camera_image_grahicscene;
    QGraphicsScene *mp_segnet_image_grahicscene;

    QGraphicsScene *mp_lrf_image_grahicscene;

public://Display Image to Qt Graphicview
    QImage Mat2QImage(cv::Mat src);
    void Display_Image(cv::Mat,QGraphicsScene*,QGraphicsView*,bool _fl_clear = false);

public slots:
    //-------------------------------------------------
    // Button
    //-------------------------------------------------
    //KINOVA
    void SlotButtonKinovaInit();
    void SlotButtonKinovaInitMotion();
    void SlotButtonKinovaAlignPanel();
    void SlotButtonKinovaDoManipulate();

    void SlotButtonKinovaMoveStepUp();
    void SlotButtonKinovaMoveStepDw();
    void SlotButtonKinovaMoveStepRi();
    void SlotButtonKinovaMoveStepLe();

    //LRF
    void SlotButtonLRFOn();

    //Camera
    void SlotButtonCameraOn();
    void SlotButtonSegnetOn(bool _check);

    //-------------------------------------------------
    // Edite Update
    //-------------------------------------------------

    void SlotEditeKinovaPosition(CartesianPosition _position);

    //-------------------------------------------------
    // View Update
    //-------------------------------------------------
    //Camera
    void SlotViewCameraImage(cv::Mat _image);
    void SlotViewSegnetImage(cv::Mat _image);

    //LRF
    void SlotViewLRFImage(cv::Mat _image);
};

#endif // MANIPULATION_DLG_H
