#ifndef CGPS_H
#define CGPS_H

#include "gps_struct.h"
#include <QtSerialPort/QSerialPort>
#include <QDataStream>

class CGPS
{
public:
    CGPS();
    ~CGPS();

    QSerialPort *port = new QSerialPort();

    StrDef_UBX_NAV_PVT m_mstr_pvt;
    StrDef_UBX_NAV_POSECEF m_mstr_posecef;


    QByteArray m_recvData;

    int m_pvt_data_len;
    int m_posecef_data_len;

    bool fl_gps_rx;

    char m_sync1;
    char m_sync2;
    char m_pvt_id;
    char m_posecef_id;

    bool GpsInit(); // port open and set gps property
    bool GpsUpdate();
    bool CheckSum(unsigned char* _rxbuff,int _data_buffer_N);
    bool RxParse2Data(unsigned char _msg_id, unsigned char* _rxbuff);
    void GpsClose();
    bool IsGPSConnected();
    void SetInitGPS();
    void SetGroundGPS();
    void SetCurrentHeading();
    double GetCurrentHeading();
    void SetInitHeading();
    double GetInitHeading();
    Gpspoint GetInitGPS();

//    Ground_Gpspoint GetGroundGpspoint(Gpspoint _init_point, double _init_heading);
    Ground_Gpspoint CalcGroundGpspoint();
    Gpspoint CalcGpspoint(double _cur_heading,double _relative_bearing, double dist, Gpspoint _cur_gps_point);
    double DistCalc_Gps2Gps(Gpspoint _cur_point, Gpspoint _target_point);
    double BearingCalc_Gps2Gps(Gpspoint _cur_point, Gpspoint _target_point);
    Ground_Bodypoint CalcBodypoint_Ground();

private:
    Gpspoint m_inital_gpspoint;
    Gpspoint m_cur_gpspoint;
    Ground_Gpspoint m_ground_gpspoints;
    Ground_Bodypoint m_ground_bodypoints;

    double m_cur_heading = 0;
    double m_init_heading = 0;
};

//gps related function


#endif // CGPS_H
