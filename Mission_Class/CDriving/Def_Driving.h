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

#endif // DEF_DRIVING_H
