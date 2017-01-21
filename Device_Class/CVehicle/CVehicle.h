#ifndef CVEHICLE_H
#define CVEHICLE_H

#include "Constants.h"
#include "ErrorCodes.h"
#include "RoboteqDevice.h"

#include <QThread>
#include <QMutex>

#include <ctime>
#include <cstdio>
#include <iostream>

#define MAX_VEL 200

using namespace std;

class CVehicle:public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CVehicle();
    ~CVehicle();

private:
    bool fl_isGO;

    bool fl_connection;

    int m_vel;
    int m_timecnt;
    int m_dir;
    int m_status;
    int m_battery;

    string mstr_response;
    RoboteqDevice mc_device;

public:
    int Connect();
    int SetControl();
    bool Move(int _dir,int _vel);
    void CheckVolt();


};

#endif // CVEHICLE_H
