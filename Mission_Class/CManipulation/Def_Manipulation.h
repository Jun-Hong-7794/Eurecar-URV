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

    double forece_threshold;

}KINOVA_FORCE_CTRL_STRUCT;

#endif // DEF_MANIPULATION_H
