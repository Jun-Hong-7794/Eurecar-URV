#ifndef CGPS_H
#define CGPS_H

//-------------------------------------------------
// Qt Dependences Class
//-------------------------------------------------
#include <QThread>
#include <QMutex>
#include <QTimer>
#include <QElapsedTimer>



#include "gps_struct.h"
#include "Device_Class/CIMU/CIMU.h"
#include <QtSerialPort/QSerialPort>
#include <QDataStream>
#include <opencv2/core.hpp>

class CGPS:public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CGPS();
    ~CGPS();

private:
    CIMU* mpc_imu;

    int m_pvt_data_len;
    int m_posecef_data_len;
    bool fl_gps_rx;
    char m_sync1;
    char m_sync2;
    char m_pvt_id;
    char m_posecef_id;
    StrDef_UBX_NAV_PVT m_mstr_pvt;
    StrDef_UBX_NAV_POSECEF m_mstr_posecef;
    QByteArray m_recvData;

    Gpspoint m_inital_gpspoint;
    double m_init_heading = 0;
    Gpspoint m_cur_gpspoint;
    double m_cur_heading = 0;
    vector<double> m_cur_heading_rad = {0, 0, 0};

    Ground_Gpspoint m_ground_gpspoints;
    Ground_Bodypoint m_ground_bodypoints;


    double rotate_angle_rad;
    vector<cv::Point2f> ground_rotated;

public:
    QSerialPort *port = new QSerialPort();

    bool running_command = true;

    bool CheckSum(unsigned char* _rxbuff,int _data_buffer_N);
    bool RxParse2Data(unsigned char _msg_id, unsigned char* _rxbuff);

    bool GpsInit(string _port); // port open and set gps property
    void GpsClose();
    bool IsGPSConnected();

    bool GpsUpdate();
    bool GPSrecieveCheck();

    void SetInitGPS();
//    void SetInitHeading();

    void SetCurrentHeading();
    void SetGroundGPS();

    double GetCurrentHeading();
    double GetInitHeading();
    Gpspoint GetCurrentGPS();
    Gpspoint GetInitGPS();
    vector<double> GetIMUEuler();

//    Ground_Gpspoint GetGroundGpspoint(Gpspoint _init_point, double _init_heading);
//    Ground_Gpspoint CalcGroundGpspoint();
    double DistCalc_Gps2Gps(Gpspoint _cur_point, Gpspoint _target_point);
    double BearingCalc_Gps2Gps(Gpspoint _cur_point, Gpspoint _target_point);
    vector<cv::Point2f> CalcBodypoint_Ground();

    bool CheckInBoundary(vector<cv::Point2f> _ground_bodypoint, double _object_x,double _object_y);

    Gpspoint CalcGpspoint(double _relative_angle, double _dist_km, Gpspoint _start_gps_point);
};


#endif // CGPS_H
