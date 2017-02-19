#ifndef MANIPULATION_DLG_H
#define MANIPULATION_DLG_H

#include <QDialog>
#include <QFile>
#include <QTextStream>
#include <QTime>
//-------------------------------------------------
// QCustomPlot
//-------------------------------------------------
#include "qcustomplot.h"

//-------------------------------------------------
// Device Class
//-------------------------------------------------
#include "Device_Class/CGPS/CGPS.h"
#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"
#include "Device_Class/CKinova/CKinova.h"
#include "Device_Class/CVehicle/CVehicle.h"
#include "Device_Class/CVelodyne/CVelodyne.h"
#include "Device_Class/CGripper/CGripper.h"

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
    // Valve Size Graph Index
    //-------------------------------------------------
    QMutex mtx_valve_data_graph;
    QMutex mtx_valve_anal_graph;

    QMutex mtx_kinova_force_vector_graph;

    int m_valve_size_graph_num;
    int m_valve_anal_graph_num;

    QVector<QVector<double>> mqvec_valve_data_x;
    QVector<QVector<double>> mqvec_valve_data_y;

    QVector<QVector<double>> mqvec_valve_anal_x;
    QVector<QVector<double>> mqvec_valve_anal_y;


    QVector<double> mqvec_kinova_force_data_time;
    QVector<double> mqvec_kinova_force_thresh_time;

    QVector<double> mqvec_kinova_force_data_x;
    QVector<double> mqvec_kinova_force_data_y;
    QVector<double> mqvec_kinova_force_data_z;

    QVector<double> mqvec_kinova_force_thresh_x;
    QVector<double> mqvec_kinova_force_thresh_y;
    QVector<double> mqvec_kinova_force_thresh_z;

    QVector<QVector<double>> mqvec_kinova_force_x_data_x;
    QVector<QVector<double>> mqvec_kinova_force_x_data_y;

    QVector<QVector<double>> mqvec_kinova_force_y_data_x;
    QVector<QVector<double>> mqvec_kinova_force_y_data_y;

    QVector<QVector<double>> mqvec_kinova_force_z_data_x;
    QVector<QVector<double>> mqvec_kinova_force_z_data_y;


    void InterpreteValveSizeDataFile(QString _file, QVector<double>& _x, QVector<double>& _y);
    bool InterpreteValveSizeDataLine(QString _line, QVector<double>& _x, QVector<double>& _y, QString& _title);

    //-------------------------------------------------
    // Mission Class
    //-------------------------------------------------
    CManipulation* mpc_manipulation;

private:// Graphic Scene
    QGraphicsScene *mp_camera_image_grahicscene;
    QGraphicsScene *mp_segnet_image_grahicscene;

    QGraphicsScene *mp_lrf_image_grahicscene;

    QGraphicsScene *mp_valve_image_grahicscene;

public://Display Image to Qt Graphicview
    QImage Mat2QImage(cv::Mat src);
    void Display_Image(cv::Mat,QGraphicsScene*,QGraphicsView*,bool _fl_clear = false);

public:
    //Set Valve Size Data
    int GetValveSizeDataGraphCurrentIndex();
    int GetValveSizeAnalGraphCurrentIndex();
    void SetValveSizeData(QCustomPlot* _plot,
                          QVector<double> _input_x, QVector<double> _input_y,
                          QVector<QVector<double>>& _contain_x,QVector<QVector<double>>& _contain_y,
                          int& _current_index, int _graph_index, QString _data_name = "Empty");

public slots:
    //-------------------------------------------------
    // Button
    //-------------------------------------------------
    //KINOVA
    void SlotButtonKinovaInit();
    void SlotButtonKinovaInitMotion();
    void SlotButtonKinovaAlignPanel();
    void SlotButtonKinovaDoManipulate();

    void SlotButtonKinovaGetPosition();

    void SlotButtonKinovaMoveStepUp();
    void SlotButtonKinovaMoveStepDw();
    void SlotButtonKinovaMoveStepRi();
    void SlotButtonKinovaMoveStepLe();
    void SlotButtonKinovaMoveStepFw();
    void SlotButtonKinovaMoveStepBw();

    void SlotButtonKinovaMoveStepRollUp();
    void SlotButtonKinovaMoveStepRollDw();
    void SlotButtonKinovaMoveStepPitchUp();
    void SlotButtonKinovaMoveStepPitchDw();

    void SlotButtonKinovaGetForceFeedBackData();
    void SlotButtonKinovaSetForceThreshData();

    void SlotButtonKinovaBaseRotate();

    //LRF
    void SlotButtonLRFOn();
    void SlotButtonGetLRFInfo();
    void SlotButtonHorizenDistance();

    //Camera
    void SlotButtonCameraOn();
    void SlotButtonSegnetOn(bool _check);

    //End Effector
    void SlotButtonEEffectorGrasp();
    void SlotButtonEEffectorPoseCheck();
    void SlotButtonEEffectorGoToOrg();

    //Gripper
    void SlotButtonGripperGrasp();
    void SlotButtonGripperTorqueOn();

    //Valve Recognition
    void SlotButtonGraphClear();
    void SlotButtonSaveGraphData();
    void SlotButtonLoadGraphData();
    void SlotButtonAnalisysGraphData();

    void SlotButtonAnalGraphClear();
    void SlotButtonAnalLoadGraphData();
    void SlotButtonAnalDataAnalisys();

    //Step Function
    void SlotButtonLRFKinovaCtrl();
    void SlotButtonKinovaForceCtrl();
    void SlotButtonEEffectorLoadCheckIter();

    //-------------------------------------------------
    // Edite Update
    //-------------------------------------------------
    void SlotEditeKinovaPosition(CartesianPosition _position);
    void SlotEditeKinovaForceVector(CartesianPosition _force_vector);

    void SlotEditeLRFKinovaAngleStruct(LRF_KINOVA_ANGLE_CTRL_STRUCT _lrf_kinova_option);
    void SlotEditeLRFKinovaHorizenStruct(LRF_KINOVA_HORIZEN_CTRL_STRUCT _lrf_kinova_option);
    void SlotEditeLRFKinovaVertivalStruct(LRF_KINOVA_VERTICAL_CTRL_STRUCT _lrf_kinova_option);

    void SlotEditeGripperStatus(GRIPPER_STATUS _gripper_status);

    //-------------------------------------------------
    // View Update
    //-------------------------------------------------
    //Camera
    void SlotViewCameraImage(cv::Mat _image);
    void SlotViewSegnetImage(cv::Mat _image);

    //Custom Graph
    void SlotValveSizeData(QVector<double>, QVector<double>, int _graph_index);
    void SlotKinovaForceVectorData(CartesianPosition _force_vector);

    //Valve Modeling
    void SlotValveImage(cv::Mat _image);

    //LRF
    void SlotViewLRFImage(cv::Mat _image);
};

#endif // MANIPULATION_DLG_H
