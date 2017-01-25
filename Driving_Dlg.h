#ifndef DRIVING_DLG_H
#define DRIVING_DLG_H

#include <QDialog>
#include <QtWidgets>

//-------------------------------------------------
// Device Class
//-------------------------------------------------
#include "opencv_header.h"

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
#include "Mission_Class/CDriving/CDriving.h"




namespace Ui {
class Driving_Dlg;
}

class Driving_Dlg : public QDialog
{
    Q_OBJECT

public:
    explicit Driving_Dlg(CDriving* _pc_driving,QWidget *parent = 0);
    ~Driving_Dlg();

private:
    Ui::Driving_Dlg *ui;

private:
    //-------------------------------------------------
    // Mission Class
    //-------------------------------------------------
    CDriving* mpc_drivig;

public:
    void InitDlg(CDriving* _p_driving);

public slots:
    //-------------------------------------------------
    // Button
    //-------------------------------------------------

    // Push Button
    void SlotButtonVehicleConnet();
    void SlotButtonVehicleOperate();
    void SlotButtonVehicleOptionSetting();

    void SlotButtonVelodyneConnet();

    // Velodyne View
    void SlotVeloyneParser(bool _parser_complete);


private slots:
    void on_rd_vehicle_dir_forward_clicked();
};

#endif // DRIVING_DLG_H




