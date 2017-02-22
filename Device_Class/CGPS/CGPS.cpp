#include "CGPS.h"
#include <iostream>
#include <string>
#include <iomanip>

#define PI 3.14159265359

CGPS::CGPS()
{
    mpc_imu = new CIMU;

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

    m_cur_gpspoint.lat = -1;
    m_cur_gpspoint.lon = -1;
    m_cur_gpspoint.height = -1;

    fl_gps_rx = false;
}

CGPS::~CGPS()
{
    running_command = false;
    this->exit();

    port->close();
    if(port->isOpen())
        std::cout << "GPS port close error" << std::endl;
    else
        delete port;
}

bool CGPS::GpsInit(string _port)
{
    port->setPortName(_port.c_str()); //need to be change

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
        this->start();
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
//    m_ground_gpspoints = CalcGroundGpspoint();
}
void CGPS::SetCurrentHeading()
{
    m_cur_heading = 0;
}
double CGPS::GetCurrentHeading()
{
    return m_cur_heading;
}
double CGPS::GetInitHeading()
{
    return m_init_heading;
}

void CGPS::SetInitGPS()
{
//    fl_gps_rx = false;

//    do
//    {
//        GpsUpdate();
//    }while(fl_gps_rx == false);

//    m_inital_gpspoint.lat = m_mstr_pvt.lat_deg_em7 / 10000000.0;
//    m_inital_gpspoint.lon = m_mstr_pvt.lon_deg_em7 / 10000000.0;
//    m_inital_gpspoint.height = m_mstr_pvt.height_mm;

//    m_inital_gpspoint.lat = m_cur_gpspoint.lat / 10000000.0;
//    m_inital_gpspoint.lon = m_cur_gpspoint.lon / 10000000.0;

    m_inital_gpspoint.lat = 37.3535;
    m_inital_gpspoint.lon = 127.4949;

//    double tmp = mpc_imu->GetEulerAngles().at(2);
    double tmp = 1;

    if(tmp < 0)
        tmp = tmp + 2*PI;

    m_init_heading = tmp * 180 / PI;
}

Gpspoint CGPS::GetInitGPS()
{
    return m_inital_gpspoint;
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

vector<cv::Point2f> CGPS::CalcBodypoint_Ground()
{

    double cur_heading = mpc_imu->GetEulerAngles().at(2);

    cur_heading = (1.0) * cur_heading * 180.0 / PI;

    if(m_init_heading != 0)
        double cur_heading_diff = -(cur_heading - m_init_heading);
    else
    {
        cout << "initial heading value is not set"<<endl;
    }

    double dist =0.;

    Gpspoint test_init;
    Gpspoint test_cur;

    test_init.lat = 37.26;
    test_init.lon = 127.59;

    test_cur.lat = 37.26;
    test_cur.lon = 127.59;

//    dist = DistCalc_Gps2Gps(m_inital_gpspoint,m_cur_gpspoint);

    double bearing=0;
//    bearing = BearingCalc_Gps2Gps(m_inital_gpspoint,m_cur_gpspoint);

    dist = DistCalc_Gps2Gps(test_init,test_cur);
    bearing = BearingCalc_Gps2Gps(test_init,test_cur);

    double shift_x,shift_y =0;

    shift_x = dist * sin((cur_heading + bearing) * PI / 180);
    shift_y = dist * cos((cur_heading + bearing) * PI / 180);

//    double rotate_angle_degree = cur_heading - m_init_heading;
    double rotate_angle_degree = cur_heading - 0;
    rotate_angle_rad = rotate_angle_degree * PI / 180;

    cv::Point2f ground[4]; //order : left lefttop righttop right


//    vector<cv::Point2f> ground_rotated; //order : left lefttop righttop right
    ground_rotated.resize(4);

    double test_ratio = 0.1;

    ground[0].x = 0 *test_ratio;
    ground[0].y =-45 *test_ratio;

    ground[1].x = -60 *test_ratio;
    ground[1].y = -45 *test_ratio;

    ground[2].x =-60 *test_ratio;
    ground[2].y =45 *test_ratio;

    ground[3].x = 0 *test_ratio;
    ground[3].y = 45 *test_ratio;

    for(int i=0; i<4; i++)
    {
        ground_rotated[i].x = ground[i].x * cos(rotate_angle_rad) - ground[i].y * sin(rotate_angle_rad) - shift_x;
        ground_rotated[i].y = ground[i].x * sin(rotate_angle_rad) + ground[i].y * cos(rotate_angle_rad) - shift_y;
    }

    return ground_rotated;
}

bool CGPS::CheckInBoundary(vector<cv::Point2f> _ground_bodypoint, double _object_x,double _object_y)
{
    cv::Point2f CheckReferPoint[4];

    for(int i = 0; i < 4 ;i++)
    {
        CheckReferPoint[i].x = ground_rotated[i].x*cos(-rotate_angle_rad) - ground_rotated[i].y*sin(-rotate_angle_rad);
        CheckReferPoint[i].y = ground_rotated[i].x*sin(-rotate_angle_rad) + ground_rotated[i].y*cos(-rotate_angle_rad);
    }

    double rotate_object_x = _object_x * cos(-rotate_angle_rad) - _object_y * sin(-rotate_angle_rad);
    double rotate_object_y = _object_x * sin(-rotate_angle_rad) + _object_y * cos(-rotate_angle_rad);

    double x_min,x_max,y_min,y_max;

    y_min = (CheckReferPoint[0].y > CheckReferPoint[3].y ? CheckReferPoint[3].y : CheckReferPoint[0].y);
    y_max = (CheckReferPoint[0].y < CheckReferPoint[3].y ? CheckReferPoint[3].y : CheckReferPoint[0].y);
    x_min = (CheckReferPoint[3].x > CheckReferPoint[2].x ? CheckReferPoint[2].x : CheckReferPoint[3].x);
    x_max = (CheckReferPoint[3].x < CheckReferPoint[2].x ? CheckReferPoint[2].x : CheckReferPoint[3].x);

    if(rotate_object_x < x_max && rotate_object_x > x_min && rotate_object_y < y_max && rotate_object_y > y_min )
        return true;
    else
        return false;
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
//            memcpy(&m_mstr_posecef,&_rxbuff[6],sizeof(m_mstr_posecef)); // data + checksum -> 20
            cout << "data id 1" << endl;
//            return true;
            fl_gps_rx = false;
            return false;
        }
        else
        {
            cout << "data error" << endl;
            fl_gps_rx = false;
            return false;
        }
    }
    else if(_msg_id == 7)
    {
        if(CGPS::CheckSum(_rxbuff,92))
        {
            cout << "data id 7" << endl;
            memcpy(&m_mstr_pvt,&_rxbuff[6], sizeof(m_mstr_pvt)); // data + checksum -> 92
//            m_cur_gpspoint.lat = m_mstr_pvt.lat_deg_em7 / 10000000.0;
//            m_cur_gpspoint.lon = m_mstr_pvt.lon_deg_em7 / 10000000.0;

            m_cur_gpspoint.lat = 37.3535;
            m_cur_gpspoint.lon = 127.4949;
            fl_gps_rx = true;
            return true;
        }
        else
        {
            cout << "data error" << endl;
            fl_gps_rx = false;
            return false;
        }
    }
    else
    {
        cout << "data error" << endl;
        fl_gps_rx = false;
        return false;
    }
}

bool CGPS::GPSrecieveCheck()
{
    return fl_gps_rx;
}

bool CGPS::GpsUpdate()
{
    if(port->isOpen())
    {
        bool ret = port->waitForReadyRead(100);

//        cout << "GPS Thread id : " << QThread::currentThreadId()<<endl;
        if (ret)
        {
            m_recvData = port->readAll();
            cout << "error"<< endl;

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
                    cout << "error"<< endl;
                    fl_gps_rx = false;
                }
                else
                {
                     fl_gps_rx = this->RxParse2Data(*(Rxbuffer+3),Rxbuffer);
                }
            }
            else
            {
                cout << "error"<< endl;
                fl_gps_rx = false;
            }

        }
    }

    m_cur_heading_rad = mpc_imu->GetEulerAngles();

    double tmp_cur_heading = m_cur_heading_rad.at(2);
    if(tmp_cur_heading < 0)
        tmp_cur_heading = tmp_cur_heading + 2*PI;

    m_cur_heading = tmp_cur_heading * 180 / PI;

    port->clear();
    m_recvData.clear();

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

vector<double> CGPS::GetIMUEuler()
{
    return m_cur_heading_rad;
}

// run thread!-------------------------------------------
//
void CGPS::run()
{
    while(running_command)
    {
        GpsUpdate();
        msleep(30);
        port->flush();
    }
}

//
//-------------------------------------------------------





