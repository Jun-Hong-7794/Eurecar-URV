//-------------------------------------------------
// Eurecar-URV(Team KAIST) is Winner of 2017 MBZIRC Challenge 2.
// This Project created by Jun Hong. 2017-01-18
//-------------------------------------------------
#ifndef EURECARURV_DLG_H
#define EURECARURV_DLG_H

#include <QMainWindow>
#include <QtWidgets>

#include "Driving_Dlg.h"
#include "Manipulation_Dlg.h"

//-------------------------------------------------
// Script Class
//-------------------------------------------------
#include "Mission_Class/CScript/CScript.h"


namespace Ui {
class EurecarURV_Dlg;
}

class EurecarURV_Dlg : public QMainWindow
{
    Q_OBJECT

public:
    explicit EurecarURV_Dlg(QWidget *parent = 0);
    ~EurecarURV_Dlg();

private:
    Ui::EurecarURV_Dlg *ui;
    Driving_Dlg *mpdlg_driving;
    Manipulation_Dlg *mpdlg_manipulation;

private:
    //-------------------------------------------------
    // Device Class
    //-------------------------------------------------
    CGPS* mpc_gps;
    CLRF* mpc_lrf;
    CCamera* mpc_camera;
    CKinova* mpc_kinova;
    CVehicle* mpc_vehicle;
    CVelodyne* mpc_velodyne;
    CGripper* mpc_gripper;

    //-------------------------------------------------
    // Mission Class
    //-------------------------------------------------
    CDriving* mpc_drivig;
    CManipulation* mpc_manipulation;

    //-------------------------------------------------
    // Script Class
    //-------------------------------------------------
    CScript* mpc_script;

public://Display Image to Qt Graphicview
    QImage Mat2QImage(cv::Mat src);
    void Display_Image(cv::Mat,QGraphicsScene*,QGraphicsView*,bool _fl_clear = false);

public slots:
    //-------------------------------------------------
    // Button
    //-------------------------------------------------

    // Menu Button
    void SlotMenuButtonDriving_Dlg();
    void SlotMenuButtonManipulation_Dlg();

    void SlotMenuButtonScenarioLoad();

    // Push Button
    void SlotButtonVelodyne();
    void SlotButtonKinova();

public:
    void ScenarioInfoDisplay();


};

#endif // EURECARURV_DLG_H
