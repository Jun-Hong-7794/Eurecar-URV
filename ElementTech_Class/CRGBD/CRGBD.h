#ifndef CRGBD_H
#define CRGBD_H

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>

#include "Device_Class/CLRF/CLRF.h"
#include "Device_Class/CCamera/CCamera.h"

#include "ElementTech_Class/CSegnet/CSegnet.h"
#include "ElementTech_Class/CSSD/CSSD.h"

#include "RGBD_Data.h"

#define THREAD_SEGNET_INDEX 1 // Segnet

#define RGBD_PI   3.14159265359
#define RGBD_D2R (RGBD_PI / 180.0)
#define RGBD_R2D (180.0/RGBD_PI)

// _mode: 0x1000(Rough mode), 0x2x0x(Precise mode-LEFT)
//                            0xx0x1(LEFT), 0xx0x1(RIGHT) 0xx1xx(Constant Mode), 0xx2xx(RANSAC Mode)
#define L_M_ROUGH               0x1200 // Basically Using RANSAC.
#define L_M_PRECISE             0x2000
#define L_M_DIR_LEFT            0x0001
#define L_M_DIR_RIGHT           0x0002
#define L_M_VALUE_CONSTANT      0x0100
#define L_M_VALUE_RANSAC        0x0200

typedef struct _Localizaion_Info_On_Panel{

    int vertical_dst;
    int horizen__dst;

    double angle;//Deg

}LOCALIZATION_INFO_ON_PANEL;

class CRGBD: public QThread{
            Q_OBJECT
protected:
    void run();

public:
    CRGBD();
    CRGBD(CLRF* _pc_lrf);
    CRGBD(CCamera* _pc_camera, CLRF* _pc_lrf, CSSD* _ssd);

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
    CSSD *mpc_ssd;
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
    void GetLRFInfo(double &_slope, double &_distance, double _s_deg = 10, double _e_deg = 170, int _inlier_lrf_dst = 800/*mm*/);
    void GetHorizenDistance(double _inlier_distance,double& _horizen_distance, double& _s_inlier_deg, double& _e_inlier_deg,
                            double& _virt_s_deg, double& _virt_e_deg, double _s_deg = 20, double _e_deg = 160, int _sampling_loop = 1);

    // _mode: 0x1000(Rough mode), 0x2x01(Precise mode-LEFT), 0x2x02(Precise mode-Righ), 0xx1xx(Constant Mode), 0xx2xx(RANSAC Mode)
    void LocalizationOnPanel(LOCALIZATION_INFO_ON_PANEL &_info,int _mode, double _s_deg = 10/*deg*/,
                             double _e_deg = 170/*deg*/, int _inlier_dst = 1100/*mm*/, int _current_v_dst = 240/*mm, for const mode*/, double _current_ang = 0/*deg for const mode*/);

    cv::Mat GetSegnetImage(cv::Mat _org_img);

    bool RGB_DThreadSetting(int _function_index);

private:
    //-------------------------------------------------
    // Calculation Function
    //-------------------------------------------------
    void ClaculateLRFHeightDistance(long* _lrf_org_data, double _s_deg, double _e_deg, int& _s_index, std::vector<POINT_PARAM>& _point_vec, int _inlier_lrf_dst = 800/*mm*/);
    LINE_PARAM EstimateLineEquation(std::vector<POINT_PARAM>& _point_vec);

    void ClaculateHorizenDistance(std::vector<POINT_PARAM>& _point_vec, double _inlier_distance,double& _horizen_distance, int& _s_inlier_inx, int& _e_inlier_inx);
    cv::Mat LRFDataToMat(std::vector<POINT_PARAM> _point_vec, double _inlier_distance, double _max_distance = 1500/*mm*/);

    void ClaculateVirtualHorizenInfo(double& _vir_s_deg, double& _vir_e_deg,
                                     double _s_x_distance, double _e_x_distance, double _vir_v_distance = 1000/*mm*/);

public:
    void SegnetFunction();

signals:
    void SignalCameraImage(cv::Mat);
    void SignalSegnetImage(cv::Mat);

    void SignalLRFMapImage(cv::Mat);
};

#endif // CRGBD_H
