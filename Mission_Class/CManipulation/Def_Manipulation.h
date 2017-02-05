#ifndef DEF_MANIPULATION_H
#define DEF_MANIPULATION_H
#include <QString>

typedef struct _LRF_Kinova_Angle_Ctrl_Struct{

    bool lrf_kinova_mission;

    double desired_distance;

    double error;

    double s_deg;
    double e_deg;

    //Output
    double slope;
    double current_distance;

}LRF_KINOVA_ANGLE_CTRL_STRUCT;

typedef struct _LRF_Kinova_Horizen_Ctrl_Struct{

    bool lrf_kinova_mission;

    //Setting
    bool sensor_option; //true: can not move kinova, Just sensing. false: can move kinova, normal mode.

    double desired_inlier_deg_avr/*deg*/;

    double error;

    double s_deg;
    double e_deg;

    int inlier_lrf_dst;

    int loop_sleep;

    //Output
    double current_h_distance;
    double inlier_deg_s_output;
    double inlier_deg_e_output;

}LRF_KINOVA_HORIZEN_CTRL_STRUCT;

typedef struct _LRF_Kinova_Vertical_Ctrl_Struct{

    bool lrf_kinova_mission;

    double desired_distance;

    double error;

    double s_deg;
    double e_deg;

    int inlier_lrf_dst;

    int loop_sleep;

    bool sensor_option; //true: can not move kinova, Just sensing. false: can move kinova, normal mode.

    //Output
    double slope;
    double current_distance;

}LRF_KINOVA_VERTICAL_CTRL_STRUCT;

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

typedef struct _Kinova_Rotate_Valve_Struct{

    bool kinova_rotate_valve_mission;

    bool init_angle;

    bool using_current_coord;

    double center_x;
    double center_y;
    double center_z;

    double theta;
    double radius;

    double forece_threshold;

}KINOVA_ROTATE_VALVE_STRUCT;

typedef struct _Gripper_Force_Ctrl_Struct{

    bool gripper_force_ctrl_mission;

    int forece_threshold;

    int bend_deg;

    int pose_1;
    int pose_2;

}GRIPPER_FORCE_CTRL_STRUCT;

typedef struct _Gripper_Magnet_Ctrl_Struct{

    bool gripper_magnet_ctrl_mission;

    bool fl_magnet; //true : On, false : Off

}GRIPPER_MAGNET_CTRL_STRUCT;

typedef struct _Manipulation_Option{

    LRF_KINOVA_VERTICAL_CTRL_STRUCT lrf_kinova_vertical_option;

    LRF_KINOVA_ANGLE_CTRL_STRUCT lrf_kinova_angle_option;

    LRF_KINOVA_HORIZEN_CTRL_STRUCT lrf_kinova_horizen_option;

    KINOVA_FORCE_CTRL_STRUCT kinova_force_option;

    KINOVA_ROTATE_VALVE_STRUCT kinova_rotate_valve_option;

    KINOVA_DO_MANIPULATE_STRUCT kinova_manipulate_option;

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_option;

    GRIPPER_MAGNET_CTRL_STRUCT gripper_magnet_option;

}MANIPULATION_OPTION;


#endif // DEF_MANIPULATION_H
