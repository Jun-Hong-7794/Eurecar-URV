#ifndef DEF_DRIVING_H
#define DEF_DRIVING_H

typedef struct _Driving_struct{

    bool driving_mission;

    int direction;
    int velocity;

}DRIVING_STRUCT;

typedef struct _Parking_struct{
    int fnc_index; // 1: Driving 2: Parking


}PARKING_STRUCT;

typedef struct _Driving_Option{

    DRIVING_STRUCT driving_option;

    PARKING_STRUCT parking_option;

}DRIVING_OPTION;


#endif // DEF_DRIVING_H
