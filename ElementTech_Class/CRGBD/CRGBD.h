#ifndef CRGBD_H
#define CRGBD_H

#include <stdio.h>
#include <stdlib.h>

#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"

#include "ElementTech_Class/CSegnet/CSegnet.h"

#include "RGBD_Data.h"

#define THREAD_SEGNET_INDEX 1 // Segnet

#define RGBD_PI   3.14159265359
#define RGBD_D2R (RGBD_PI / 180.0)
#define RGBD_R2D (180.0/RGBD_PI)

class CRGBD: public QThread{
            Q_OBJECT
protected:
    void run();

public:
    CRGBD();
    CRGBD(CCamera* _pc_camera, CLRF* _pc_lrf);

    ~CRGBD();
private:
    long mary_lrf_distance[NUMBER_OF_LRF_POINTS];

private:
    char fl_function_index; // 1 : segnet

private:
    std::vector<POINT_PARAM> mvec_point_vector;
    std::vector<LINE_PARAM> mvec_line_eq_vector;

private:
    //-------------------------------------------------
    // Element Tech Class
    //-------------------------------------------------
    CSegnet mc_segnet;
    //-------------------------------------------------
    // Device Class
    //-------------------------------------------------
    CLRF* mpc_lrf;
    CCamera* mpc_camera;

public:
    //-------------------------------------------------
    // Element Tech Function
    //-------------------------------------------------

    // LRF Range(Angle) is redefined by jun.
    // Originaly, The Range is 0 deg to 270 deg.
    // But 0 ~ 45 deg and 225 ~ 270 deg dose not necessary our robot.(Becuase, That range is backward.)
    // So, in our system, we redefind the range(45deg => 0deg deg, 225deg => 180deg)
    void GetLRFInfo(double &_slope, double &_distance, double _s_deg = 0, double _e_deg = 180);

    cv::Mat GetSegnetImage(cv::Mat _org_img);

    bool RGB_DThreadSetting(int _function_index);

private:
    //-------------------------------------------------
    // Calculation Function
    //-------------------------------------------------
    void ClaculateLRFHeightDistance(long* _lrf_org_data, double _s_deg);
    void EstimateLineEquation();

public:
    void SegnetFunction();

signals:
    void SignalCameraImage(cv::Mat);
    void SignalSegnetImage(cv::Mat);

    void SignalLRFMapImage(cv::Mat);
};

#endif // CRGBD_H
