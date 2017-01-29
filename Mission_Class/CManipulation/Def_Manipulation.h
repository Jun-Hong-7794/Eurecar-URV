#ifndef DEF_MANIPULATION_H
#define DEF_MANIPULATION_H

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

typedef struct _LRF_Vehicle_Struct{

    bool lrf_vehicle_mission;

    double inlier_distance;
    double horizen_distance;

    double s_inlier_deg;
    double e_inlier_deg;

}LRF_VEHICLE_STRUCT;

typedef struct _Manipulation_Option{

    LRF_KINOVA_STRUCT lrf_kinova_option;

    LRF_VEHICLE_STRUCT lrf_vehicle_option;

    KINOVA_FORCE_CTRL_STRUCT kinova_force_option;

    KINOVA_DO_MANIPULATE_STRUCT kinova_manipulate_option;

    GRIPPER_FORCE_CTRL_STRUCT gripper_force_option;

}MANIPULATION_OPTION;


#endif // DEF_MANIPULATION_H
