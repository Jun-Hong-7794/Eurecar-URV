#ifndef DEF_DRIVING_H
#define DEF_DRIVING_H
#include <QString>

typedef struct _Driving_struct{

    bool driving_mission;

    int direction;
    double velocity;

    int drive_left;
    int drive_differ_left;
    int drive_right;
    int drive_differ_right;

    int parking_left;
    int parking_differ_left;
    int parking_right;
    int parking_differ_right;
    int parking_forward;
    int parking_backward;

    double panel_center_check_point;

    double parking_dist;
    double parking_thres;

    int max_vel_script;
    int min_vel_script;

    double side_center_margin;

}DRIVING_STRUCT;

typedef struct _Parking_struct{
    int fnc_index; // 1: Driving 2: Parking


}PARKING_STRUCT;

typedef struct _VEHICLE_LOCALIZATION_ON_PANEL_STRUCT{
    double desired_h_dst; // 1: Driving 2: Parking
}VEHICLE_LOCALIZATION_ON_PANEL_STRUCT;

typedef struct _Wrench_Recog_STRUCT{

    QString str_desired_wrench_index;
    double desired_wrench_index;
    QString str_lrf_v_dst;
    double lrf_v_dst;
}WRENCH_RECOGNITION_STRUCT;

typedef struct _Parking_Retry_Struct{

    double bias;
    /*m*/
    double desired_dst;

    double max_dst;
    double min_dst;

    double error_bound;

    QString str_bias;

}PARKING_RETRY_STRUCT;

typedef struct _LRF_Vehicle_Horizen_Struct{

    bool lrf_vehicle_mission;

    //Get From LRF
    double horizen_distance;
    double s_inlier_deg;
    double e_inlier_deg;

    double s_virtual_deg;
    double e_virtual_deg;
    /////////////////////

    //User Setting
    double inlier_distance;
    double desired_avr_inlier_deg;//average of inlier degree
    double desired_avr_virtual_deg;//average of inlier degree
    double error_deg_boundary;//average of inlier degree

    double velocity;

    double s_deg;
    double e_deg;

    bool sensor_option;
    /////////////////////

}LRF_VEHICLE_HORIZEN_STRUCT;

typedef struct _LRF_Vehicle_Angle_Struct{

    bool lrf_vehicle_mission;

    //Get From LRF
    double angle;
    double vertical_distance;
    /////////////////////

    //User Setting
    double desired_angle;
    double error_boundary;//error boundary

    double s_deg;
    double e_deg;

    double velocity;

    bool sensor_option;
    /////////////////////

}LRF_VEHICLE_ANGLE_STRUCT;

typedef struct _Driving_Option{

    DRIVING_STRUCT driving_option;

    PARKING_STRUCT parking_option;

    VEHICLE_LOCALIZATION_ON_PANEL_STRUCT vehicle_localization_option;

    WRENCH_RECOGNITION_STRUCT wrench_recog_option;

    PARKING_RETRY_STRUCT parking_retry_option;

    LRF_VEHICLE_ANGLE_STRUCT lrf_vehicle_angle_option;

    LRF_VEHICLE_HORIZEN_STRUCT lrf_vehicle_horizen_option;

}DRIVING_OPTION;

#endif // DEF_DRIVING_H
