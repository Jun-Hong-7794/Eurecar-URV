#include "EurecarURV_Dlg.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    EurecarURV_Dlg w;
    w.show();

    return a.exec();
}
