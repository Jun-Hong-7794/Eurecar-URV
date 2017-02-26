#ifndef DEF_MANIPULATION_H
#define DEF_MANIPULATION_H
#include <QString>

typedef struct _LRF_Sensing_Info_Struct{

    bool fl_lrf_sensing;

    int mode;

    double s_deg;
    double e_deg;

    double inlier_distance;

    //For Constant Mode
    int current_dst;
    int current_ang;

}LRF_SENSING_INFO_STRUCT;

// LRF - KINOVA
typedef struct _LRFK_VCtrl_Struct{

    bool fl_only_sensing_moving;

    bool fl_force_option;

    double force_x;
    double force_y;
    double force_z;

    int desired_v_dst;

    double error;

    int loop_sleep; /*msec*/

    LRF_SENSING_INFO_STRUCT lrf_info_struct;

}LRF_K_V_CTRL_STRUCT;

typedef struct _LRFK_HCtrl_Struct{

    bool fl_only_sensing_moving;

    int desired_h_location;

    QString wrench_hanger_index_str/*1~6*/;
    int wrench_hanger_index/*1~6*/;

    int wrench_location_1/*mm*/;
    int wrench_location_2/*mm*/;
    int wrench_location_3/*mm*/;
    int wrench_location_4/*mm*/;
    int wrench_location_5/*mm*/;
    int wrench_location_6/*mm*/;

    int error;

    int loop_sleep; /*msec*/

    LRF_SENSING_INFO_STRUCT lrf_info_struct;

}LRF_K_H_CTRL_STRUCT;

typedef struct _LRFK_ACtrl_Struct{

    bool fl_only_sensing_option;

    double desired_angle;

    double unit_deg;

    double error;

    int loop_sleep;

    LRF_SENSING_INFO_STRUCT lrf_info_struct;

}LRF_K_A_CTRL_STRUCT;
//---------------------------------

// LRF - Vehicle
typedef struct _LRFV_ACtrl_Struct{

    double desired_angle;

    double error;

    double velocity;

    int loop_sleep;

    LRF_SENSING_INFO_STRUCT lrf_info_struct;

}LRF_V_A_CTRL_STRUCT;

typedef struct _LRFV_HCtrl_Struct{

    int desired_h_location;

    double error;

    double velocity;

    int loop_sleep;

    LRF_SENSING_INFO_STRUCT lrf_info_struct;

}LRF_V_H_CTRL_STRUCT;
//---------------------------------

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

typedef struct _LRF_KINOVA_Vehicle_Horizen_Struct{

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

}LRF_K_VEHICLE_HORIZEN_STRUCT;

typedef struct _LRF_KINOVA_Vehicle_Angle_Struct{

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

}LRF_K_VEHICLE_ANGLE_STRUCT;

typedef struct _LRF_Kinova_Wrench_Location_Struct{

    bool lrf_kinova_mission;

    //Setting
    bool sensor_option; //true: can not move kinova, Just sensing. false: can move kinova, normal mode.

    QString wrench_hanger_index_str/*1~6*/;
    int wrench_hanger_index/*1~6, 7: other*/;

    double desired_start_deg/*deg*/;
    double desired_end_deg/*deg*/;

    double wrench_rel_location_1/*CartesianY_Pose*/;
    double wrench_rel_location_2/*CartesianY_Pose*/;
    double wrench_rel_location_3/*CartesianY_Pose*/;
    double wrench_rel_location_4/*CartesianY_Pose*/;
    double wrench_rel_location_5/*CartesianY_Pose*/;
    double wrench_rel_location_6/*CartesianY_Pose*/;

    double error;

    double s_deg;
    double e_deg;

    int inlier_lrf_dst;

    int loop_sleep;

    //Output
    double current_h_distance;
    double inlier_deg_s_output;
    double inlier_deg_e_output;

}LRF_KINOVA_WRENCH_LOCATION_STRUCT;

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
    bool fl_kinova_force_ctrl_sensing_option;

    int step_count;

    double move_step_x;
    double move_step_y;
    double move_step_z;

    int threshold_mode; //Mode1 : Bigger than, 2: Smaller than, 3: Range

    double force_threshold;

    double force_threshold_x;//Relative Force
    double force_threshold_y;//Relative Force
    double force_threshold_z;//Relative Force

    double force_range_s;
    double force_range_e;

    double position_limit_x;
    double position_limit_y;
    double position_limit_z;

}KINOVA_FORCE_CTRL_STRUCT;

typedef struct _Kinova_Force_Check_Struct{

    QString str_result_variable;

    bool kinova_force_ctrl_mission;
    bool fl_kinova_force_sensing_option;

    double force_threshold_x;
    double force_threshold_y;
    double force_threshold_z;

    int check_count;
    int check_threshold;

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

typedef struct _Kinova_Fit_To_Valve_Pose_Struct{

    //Setting
    QString str_valve_size;
    QString str_rotation_angle;

    int valve_size;/*16 ~ 24mm*/
    double valve_rotation_angle;

    int angle_step;
    double move_step;

}KINOVA_FIT_TO_VALVE_POSE_STRUCT;


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
    QString str_size_result_variable;
    QString str_rotation_result_variable;
    int valve_size;
    double valve_rotation_angle;

}GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT;

typedef struct _Gripper_Force_Ctrl_Struct{

    bool gripper_force_ctrl_mission;

    int force_threshold;

    int bend_deg;

    int pose_1;
    int pose_2;

}GRIPPER_FORCE_CTRL_STRUCT;

typedef struct _Gripper_Go_To_Rel_Pose_Struct{

    bool gripper_go_to_rel_pose;

    int force_threshold;

    int pose_1;
    int pose_2;

}GRIPPER_GO_TO_REL_POSE_STRUCT;

typedef struct _Gripper_Magnet_Ctrl_Struct{

    bool gripper_magnet_ctrl_mission;

    bool fl_magnet; //true : On, false : Off

}GRIPPER_MAGNET_CTRL_STRUCT;

typedef struct _Wrench_Recognition{

    bool wrench_recognition_mission;

    //Setting
    int valve_size;
    int num_of_wrench;
    int loop_count;
    QString str_valve_size;
    QString str_num_of_wrench;

    //Output
    int wrench_location;// 1 ~ 6
    QString str_result_variable;

}WRENCH_RECOGNITION;

typedef struct _Manipulation_Option{

    /*New LRF Kinova Control*/
    // Kinova
    LRF_K_V_CTRL_STRUCT lrf_k_v_ctrl_struct;

    LRF_K_H_CTRL_STRUCT lrf_k_h_ctrl_struct;

    LRF_K_A_CTRL_STRUCT lrf_k_a_ctrl_struct;

    // Vehicle
    LRF_V_H_CTRL_STRUCT lrf_v_h_ctrl_struct;

    LRF_V_A_CTRL_STRUCT lrf_v_a_ctrl_struct;
    //--------------------------------------

    LRF_KINOVA_VERTICAL_CTRL_STRUCT lrf_kinova_vertical_option;

    LRF_KINOVA_ANGLE_CTRL_STRUCT lrf_kinova_angle_option;

    LRF_KINOVA_HORIZEN_CTRL_STRUCT lrf_kinova_horizen_option;

    LRF_K_VEHICLE_ANGLE_STRUCT lrf_k_vehicle_angle_option;

    LRF_K_VEHICLE_HORIZEN_STRUCT lrf_k_vehicle_horizen_option;

    LRF_KINOVA_WRENCH_LOCATION_STRUCT lrf_kinova_wrench_option;

    KINOVA_FORCE_CTRL_STRUCT kinova_force_option;

    KINOVA_FORCE_CHECK_STRUCT kinova_force_check_option;

    KINOVA_ROTATE_VALVE_STRUCT kinova_rotate_valve_option;

    KINOVA_DO_MANIPULATE_STRUCT kinova_manipulate_option;

    KINOVA_FIT_TO_VALVE_POSE_STRUCT kinova_fit_to_valve_pose;

    GRIPPER_GO_TO_REL_POSE_STRUCT grippper_go_to_rel_pose_option;

    GRIPPER_KINOVA_VALVE_SIZE_RECOG_STRUCT gripper_kinova_valve_recog_option;

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_option;

    GRIPPER_MAGNET_CTRL_STRUCT gripper_magnet_option;

    WRENCH_RECOGNITION wrench_recognition_option;

}MANIPULATION_OPTION;


#endif // DEF_MANIPULATION_H
