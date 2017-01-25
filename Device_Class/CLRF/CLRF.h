#ifndef CLRF_H
#define CLRF_H

#include <QtWidgets>
#include <QThread>

#include <iostream>

//LRF
#include "urg_sensor.h"
#include "urg_utils.h"

//OpenCv
#include "../../opencv_header.h"

#define MAX_LRF_RANGE 1000// 1m

#define UST_20LX 1
#define UTM_30LX 2
/*
 * Hokuyo UTM_30LX
 * Angle Resolution : 0.25(deg)(0 ~ 270deg = 1080 step)
 */

#define ANGLE_RESOLUTION 0.25
#define NUMBER_OF_LRF_POINTS 1081

class CLRF: public QThread{
    Q_OBJECT
protected:
    void run();

public:
    CLRF();
    ~CLRF();
private:
    QMutex mtx_LRF_data;

private:
    bool fl_stream;
    bool fl_lrf_init;

    urg_t m_urg;

    long *mp_distance_data;
    unsigned short *mp_intensity_data;

    int m_number_of_point;

    int m_device_type;// UST_20LX Or UTM_30LX
    // Connects to the sensor via Ethernet and receives range data
    char* mp_connect_address;
    long m_connect_port;

    // Connects to the sensor via Serial and receives range data
    char* mp_connect_device;
    long m_baudrate;

public:
    bool CalculateColorStep(unsigned int _max_num,unsigned int _current_num, int &_red,int &_green,int &_blue);

    void LRFDataToMat(cv::Mat &_lrf_map);
    cv::Mat LRFDataToMat(int _s_index = 45, int _e_index = 225);

public:
    bool InitLRF(char* _dev_path = (char *)"/dev/ttyACM0",int _dev_type = UTM_30LX);
    bool LRFOpen(int _dev_type = UTM_30LX);

    void CloseLRF();

    bool IsLRFOn();

    int GetDeviceType();

    bool GetLRFData(long *_distance_data, unsigned short *_intensity_data = NULL) ;

signals:
    void SignalLRFImage(cv::Mat);


};

#endif // CLRF_H
