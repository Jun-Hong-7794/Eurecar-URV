#include "Manipulation_Dlg.h"
#include "ui_Manipulation_Dlg.h"

Manipulation_Dlg::Manipulation_Dlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Manipulation_Dlg)
{
    ui->setupUi(this);

    //Connetion
}

Manipulation_Dlg::~Manipulation_Dlg()
{
    delete ui;
}

void Manipulation_Dlg::InitDlg(CManipulation* _p_manipulation){
    mpc_manipulation = _p_manipulation;
}
