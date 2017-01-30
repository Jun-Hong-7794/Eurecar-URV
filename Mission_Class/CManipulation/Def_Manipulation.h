#ifndef DEF_MANIPULATION_H
#define DEF_MANIPULATION_H
#include <QString>

typedef struct _LRF_Kinova_Struct{

    bool lrf_kinova_mission;

    double desired_distance;

    double error;

    double s_deg;
    double e_deg;

}LRF_KINOVA_STRUCT;

typedef struct _Kinova_Force_Ctrl_Struct{

    bool kinova_force_ctrl_mission;

    int step_count;

    double move_step_x;
    double move_step_y;
    double move_step_z;

    double forece_threshold;

}KINOVA_FORCE_CTRL_STRUCT;

typedef struct _Kinova_Do_Manipulate_Struct{

    bool kinova_do_manipulate_mission;

    QString str_x;
    QString str_y;
    QString str_z;

    QString str_roll;
    QString str_pitch;
    QString str_yaw;

    double x;
    double y;
    double z;

    double roll;
    double pitch;
    double yaw;

    double forece_threshold;

}KINOVA_DO_MANIPULATE_STRUCT;

typedef struct _Gripper_Force_Ctrl_Struct{

    bool gripper_force_ctrl_mission;

    int forece_threshold;

    int bend_deg;

}GRIPPER_FORCE_CTRL_STRUCT;

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

typedef struct _Manipulation_Option{

    LRF_KINOVA_STRUCT lrf_kinova_option;

    LRF_VEHICLE_HORIZEN_STRUCT lrf_vehicle_horizen_option;

    LRF_VEHICLE_ANGLE_STRUCT lrf_vehicle_angle_option;

    KINOVA_FORCE_CTRL_STRUCT kinova_force_option;

    KINOVA_DO_MANIPULATE_STRUCT kinova_manipulate_option;

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_option;

}MANIPULATION_OPTION;


#endif // DEF_MANIPULATION_H
