#ifndef CRGBD_H
#define CRGBD_H

#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"

#include "ElementTech_Class/CSegnet/CSegnet.h"

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

};

#endif // CRGBD_H
