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

    QString wrench_hanger_index_str/*1~6*/;
    int wrench_hanger_index/*1~6*/;

    double wrench_location_deg_1/*deg*/;
    double wrench_location_deg_2/*deg*/;
    double wrench_location_deg_3/*deg*/;
    double wrench_location_deg_4/*deg*/;
    double wrench_location_deg_5/*deg*/;
    double wrench_location_deg_6/*deg*/;

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

    QString str_result_variable;

    bool kinova_force_ctrl_mission;

    int step_count;

    double move_step_x;
    double move_step_y;
    double move_step_z;

    double force_threshold;

    double force_threshold_x;//Relative Force
    double force_threshold_y;//Relative Force
    double force_threshold_z;//Relative Force

    double position_limit_x;
    double position_limit_y;
    double position_limit_z;

}KINOVA_FORCE_CTRL_STRUCT;

typedef struct _Kinova_Force_Check_Struct{

    bool kinova_force_ctrl_mission;

    double force_threshold_x;
    double force_threshold_y;
    double force_threshold_z;

    int check_count;

}KINOVA_FORCE_CHECK_STRUCT;

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

    double force_threshold;

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

    double force_threshold;

}KINOVA_ROTATE_VALVE_STRUCT;

typedef struct _Gripper_Kinova_Valve_Size_Recognition_Struct{

    bool gripper_kinova_valve_size_recog_mission;

    // Setting
    double grasp_pose_1;
    double grasp_pose_2;
    double release_pose_1;
    double release_pose_2;
    double force_threshold;
    double unit_rotation_angle;// CW: > 0, CCW: < 0

    int trial;//Recomand 36trial
    double rotation_angle;//Recomand 180deg

    int inlier_error;

    // Output
    int wrench_size;
    double wrench_rotation_angle;

}GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT;

typedef struct _Gripper_Force_Ctrl_Struct{

    bool gripper_force_ctrl_mission;

    int force_threshold;

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

    KINOVA_FORCE_CHECK_STRUCT kinova_force_check_option;

    KINOVA_ROTATE_VALVE_STRUCT kinova_rotate_valve_option;

    KINOVA_DO_MANIPULATE_STRUCT kinova_manipulate_option;

    GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT gripper_kinova_valve_recog_option;

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_option;

    GRIPPER_MAGNET_CTRL_STRUCT gripper_magnet_option;

}MANIPULATION_OPTION;


#endif // DEF_MANIPULATION_H
