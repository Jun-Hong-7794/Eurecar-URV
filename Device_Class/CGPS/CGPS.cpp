#include "CGPS.h"
#include <iostream>
#include <string>
#include <iomanip>

#define PI 3.14159265359

CGPS::CGPS()
{
    m_mstr_pvt = {};
    m_mstr_posecef = {};

    m_pvt_data_len = 92;
    m_posecef_data_len = 20;

    m_sync1 = 0xb5;
    m_sync2 = 0x62;

    m_pvt_id = 1;
    m_posecef_id = 7;

    m_inital_gpspoint.lat = -1;
    m_inital_gpspoint.lon = -1;
    m_inital_gpspoint.height = -1;

    fl_gps_rx = false;
}

CGPS::~CGPS()
{
    port->close();
    if(port->isOpen())
        std::cout << "GPS port close error" << std::endl;
    else
        delete port;
}

bool CGPS::GpsInit()
{
    port->setPortName("/dev/ttyACM0"); //need to be change

    port->setBaudRate(QSerialPort::Baud9600);
    port->setDataBits(QSerialPort::Data8);
    port->setParity(QSerialPort::NoParity);
    port->setStopBits(QSerialPort::OneStop);
    port->setFlowControl(QSerialPort::NoFlowControl);

    if(!port->isOpen())
        port->open(QIODevice::ReadWrite);

    if(port->isOpen())
    {
        std::cout << "gps port open and init success" << std::endl;
        return true;
    }
    else
    {
        std::cout << "gps port open failed" << std::endl;
        return false;
    }

}
bool CGPS::IsGPSConnected()
{
    if(port->isOpen())
        return true;
    else
        return false;
}


void CGPS::SetGroundGPS()
{
    m_ground_gpspoints = CalcGroundGpspoint();
}
void CGPS::SetCurrentHeading()
{
    m_cur_heading = 0;
}
double CGPS::GetCurrentHeading()
{
    return m_cur_heading;
}
void CGPS::SetInitHeading()
{
    m_init_heading = 0;
}
double CGPS::GetInitHeading()
{
    return m_init_heading;
}

void CGPS::SetInitGPS()
{
    fl_gps_rx = false;

    do
    {
        GpsUpdate();

        m_inital_gpspoint.lat = m_mstr_pvt.lat_deg_em7 / 10000000.0;
        m_inital_gpspoint.lon = m_mstr_pvt.lon_deg_em7 / 10000000.0;
        m_inital_gpspoint.height = m_mstr_pvt.height_mm;
    }while(fl_gps_rx == true);

    std::cout << std::setprecision(20) <<  m_inital_gpspoint.lat << " " << std::setprecision(20) << m_inital_gpspoint.lon << std::endl;
}

Gpspoint CGPS::GetInitGPS()
{
    return m_inital_gpspoint;
}

Ground_Gpspoint CGPS::CalcGroundGpspoint()
{
    m_ground_gpspoints.left = CalcGpspoint(m_init_heading,-90,0.045,m_inital_gpspoint);
    m_ground_gpspoints.lefttop = CalcGpspoint(m_init_heading,-36.86989,0.075,m_inital_gpspoint);
    m_ground_gpspoints.righttop = CalcGpspoint(m_init_heading,36.86989,0.075,m_inital_gpspoint);
    m_ground_gpspoints.right = CalcGpspoint(m_init_heading,90,0.045,m_inital_gpspoint);

    std::cout << m_ground_gpspoints.left.lat << ", " << m_ground_gpspoints.left.lon << std::endl;
    std::cout << m_ground_gpspoints.lefttop.lat << ", " << m_ground_gpspoints.lefttop.lon << std::endl;
    std::cout << m_ground_gpspoints.righttop.lat << ", " << m_ground_gpspoints.righttop.lon << std::endl;
    std::cout << m_ground_gpspoints.right.lat << ", " << m_ground_gpspoints.right.lon << std::endl;

    return m_ground_gpspoints;
}

Gpspoint CGPS::CalcGpspoint(double _cur_heading,double _relative_bearing, double dist, Gpspoint _cur_gps_point)
{
    const double radiusEarthKilometres = 6371.01;
    double distRatio = dist / radiusEarthKilometres;
    double distRatioSine = sin(distRatio);
    double distRatioCosine = cos(distRatio);

    double startLatRad = _cur_gps_point.lat * PI / 180;
    double startLonRad = _cur_gps_point.lon * PI / 180;

    double startLatCos = cos(startLatRad);
    double startLatSin = sin(startLatRad);

    double initialBearingRadians = (_cur_heading + _relative_bearing)* PI / 180; // current gps heading + velodyne relative bearing

    double endLatRads = asin((startLatSin * distRatioCosine) + (startLatCos * distRatioSine * cos(initialBearingRadians)));

    double endLonRads = startLonRad + atan2(sin(initialBearingRadians) * distRatioSine * startLatCos, distRatioCosine - startLatSin * sin(endLatRads));

    Gpspoint end_point;
    end_point.lat = (endLatRads) * 180 / PI; // degree
    end_point.lon = (endLonRads) * 180 / PI;

    return end_point;
}

double CGPS::DistCalc_Gps2Gps(Gpspoint _cur_point, Gpspoint _target_point)
{
    double dlon = (_target_point.lon - _cur_point.lon) * PI / 180.0;
    double dlat = (_target_point.lat - _cur_point.lat) * PI / 180.0;

    double a = pow(sin(dlat / 2.0), 2) + cos(_cur_point.lat * PI / 180.0) * cos(_target_point.lat * PI / 180.0) * pow(sin(dlon / 2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = 6367 * c;

    return d;
}

double CGPS::BearingCalc_Gps2Gps(Gpspoint _cur_point, Gpspoint _target_point)
{
    double dlon = (_target_point.lon  * PI / 180 - _cur_point.lon  * PI / 180);
    double y = sin(dlon) * cos(_target_point.lat * PI / 180);
    double x = cos(_cur_point.lat * PI / 180) * sin(_target_point.lat * PI / 180) - sin(_cur_point.lat * PI / 180)*cos(_target_point.lat * PI / 180)*cos(dlon);


    double dPhi = log(tan(_target_point.lat * PI / 180 / 2.0 + PI / 4.0) / tan(_cur_point.lat * PI / 180 / 2.0 + PI / 4.0));
    if (abs(dlon) > PI)
    {
        if (dlon > 0.0)
            dlon = -(2.0 * PI - dlon);
        else
            dlon = (2.0 * PI + dlon);
    }

    double bearing = ((atan2(y, x) * 180 / PI));

    if (bearing < 0)
        bearing = bearing + 360;

    return bearing;
}

Ground_Bodypoint CGPS::CalcBodypoint_Ground()
{
    Gpspoint _cur_point = m_cur_gpspoint;
    double _cur_heading =0 ;
    double _init_heading = m_init_heading;
    Ground_Gpspoint _gps_ground = m_ground_gpspoints;

    double dheading = _cur_heading - _init_heading;

    m_ground_bodypoints.dist_left = DistCalc_Gps2Gps(_cur_point, _gps_ground.left);
    m_ground_bodypoints.angle_left = fmod(BearingCalc_Gps2Gps(_cur_point, _gps_ground.left) + _cur_heading, 360.0);


    m_ground_bodypoints.dist_lefttop = DistCalc_Gps2Gps(_cur_point, _gps_ground.lefttop);
    m_ground_bodypoints.angle_lefttop = fmod(BearingCalc_Gps2Gps(_cur_point, _gps_ground.lefttop) + _cur_heading, 360.0);

    m_ground_bodypoints.dist_righttop = DistCalc_Gps2Gps(_cur_point, _gps_ground.righttop);
    m_ground_bodypoints.angle_righttop = fmod(BearingCalc_Gps2Gps(_cur_point, _gps_ground.righttop) + _cur_heading, 360.0);

    m_ground_bodypoints.dist_right = DistCalc_Gps2Gps(_cur_point, _gps_ground.right);
    m_ground_bodypoints.angle_right = fmod(BearingCalc_Gps2Gps(_cur_point, _gps_ground.right) + _cur_heading, 360.0);

    return m_ground_bodypoints;
}





bool CGPS::CheckSum(unsigned char* _rxbuff,int _data_buffer_N)
{
    unsigned char check_a=0;
    unsigned char check_b=0;

    for(int i=0; i<_data_buffer_N+4; i++)
    {
        check_a = check_a + _rxbuff[2 + i];
        check_b = check_b + check_a;
    }
    if(check_a == _rxbuff[_data_buffer_N+6] && check_b == _rxbuff[_data_buffer_N+7])
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool CGPS::RxParse2Data(unsigned char _msg_id, unsigned char *_rxbuff)
{
    if(_msg_id == 1)
    {
        if(CGPS::CheckSum(_rxbuff,20))
        {
            memcpy(&m_mstr_posecef,&_rxbuff[6],sizeof(m_mstr_posecef)); // data + checksum -> 20
//            return true;
            fl_gps_rx = false;
            return false;
        }
        else
        {
            fl_gps_rx = false;
            return false;
        }
    }
    else if(_msg_id == 7)
    {
        if(CGPS::CheckSum(_rxbuff,92))
        {
            memcpy(&m_mstr_pvt,&_rxbuff[6], sizeof(m_mstr_pvt)); // data + checksum -> 92
            m_cur_gpspoint.lat = m_mstr_pvt.lat_deg_em7;
            m_cur_gpspoint.lon = m_mstr_pvt.lon_deg_em7;
            fl_gps_rx = true;
            return true;
        }
        else
        {
            fl_gps_rx = false;
            return false;
        }
    }
    else
    {
        fl_gps_rx = false;
        return false;
    }
}

bool CGPS::GpsUpdate()
{
    if(port->isOpen())
    {
        bool ret = port->waitForReadyRead(1200);

        if (ret)
        {
            m_recvData = port->readAll();

            std::string start_index = std::to_string(char(0xb5)) + std::to_string(char(0x62));
            QList<QByteArray> list = m_recvData.split('\r\n');
            QList<QByteArray> list2 = list[0].split(0xb5);

            if(list2.length() > 1)
            {
                list2[1].insert(0,0xb5);

                unsigned char Rxbuffer[list2[1].size()];

                memcpy(Rxbuffer,(list2[1].data()),list2[1].size());

                if(Rxbuffer[0] != 0xb5 || Rxbuffer[1] != 0x62 || Rxbuffer[2] != 0x01) // check sync char1,2 & msg class 1
                {
                    fl_gps_rx = false;
                }
                else
                {
                     fl_gps_rx = this->RxParse2Data(*(Rxbuffer+3),Rxbuffer);
                }
            }
            else
            {
                fl_gps_rx = false;
            }

        }
    }
    return fl_gps_rx;
}

void CGPS::GpsClose()
{
    if(port->isOpen())
    {
        port->close();
        std::cout << "Gps port close" << std::endl;
    }
}




