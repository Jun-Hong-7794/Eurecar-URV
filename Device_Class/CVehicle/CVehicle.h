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
#include <sys/time.h>

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

    int m_enc_left;
    int m_enc_right;
    vector<int> m_enc_val = {0,0};

    double m_current_heading = 0.0;
    double m_reference_heading = 0.0;

    int m_past_direction_command = -1;
    timeval m_cur_time;
    long m_reference_timestamp;
    long m_current_timestamp;

    string mstr_response;
    RoboteqDevice mc_device;

    QMutex mtx_vehicle;

public:
    bool IsConnected();
    int Connect(char* _dev_path = (char *)"/dev/ttyACM0");
    void Disconnect();
    int SetControl();
    bool Move(int _dir,int _vel);
    void CheckVolt();
    void CheckEncoderValue();


    bool InitEncoder();
    vector<int> GetEncoderValue();
    vector<int> GetEncoderValueRaw();
    int CalcDistToEnc_m(double _dist_meter);


    bool ActiveMagnet(bool _on_off);
    int GetVel();

public slots:
    void SlotVehicleHeading(double _heading);

};

#endif // CVEHICLE_H
