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
    explicit Manipulation_Dlg(QWidget *parent = 0);
    ~Manipulation_Dlg();

private:
    Ui::Manipulation_Dlg *ui;

private:
    //-------------------------------------------------
    // Mission Class
    //-------------------------------------------------
    CManipulation* mpc_manipulation;

public:
    void InitDlg(CManipulation* _p_manipulation);


};

#endif // MANIPULATION_DLG_H
