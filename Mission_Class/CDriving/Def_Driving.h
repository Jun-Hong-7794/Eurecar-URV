#ifndef DEF_DRIVING_H
#define DEF_DRIVING_H

#define DR_LRF_VEHICLE_ANGLE_CONTROL         0x1000
#define DR_LRF_VEHICLE_HORIZEN_CONTROL       0x1001

typedef struct _Driving_struct{

    bool driving_mission;

    int direction;
    int velocity;

}DRIVING_STRUCT;

typedef struct _Parking_struct{
    int fnc_index; // 1: Driving 2: Parking


}PARKING_STRUCT;

typedef struct _LRF_Vehicle_Horizen_Struct{

    bool lrf_vehicle_mission;

    //Get From LRF
    double horizen_distance;

    double s_inlier_deg;
    double e_inlier_deg;
    /////////////////////

    //User Setting
    double inlier_distance;
    double desired_avr_inlier_deg;//average of inlier degree
    double error_deg_boundary;//average of inlier degree

    double velocity;
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
    /////////////////////

}LRF_VEHICLE_ANGLE_STRUCT;

typedef struct _Driving_Option{

    DRIVING_STRUCT driving_option;

    PARKING_STRUCT parking_option;

    LRF_VEHICLE_ANGLE_STRUCT lrf_vehicle_angle_option;

    LRF_VEHICLE_HORIZEN_STRUCT lrf_vehicle_horizen_option;

}DRIVING_OPTION;

#endif // DEF_DRIVING_H
