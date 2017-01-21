#ifndef VELODYNE_PARSER_H
#define VELODYNE_PARSER_H

#define VELODYNE_DATA_SIZE 1206
#define VELODYNE_BOLCKS_NUM 12
#define VELODYNE_LASERS_NUM 32

#define VELODYNE_VER_START  10.67
#define VELODYNE_VER_END   -30.67
#define VELODYNE_VER_RESOL   1.33

#define VELODYNE_HOR_START  0
#define VELODYNE_HOR_END   360
#define VELODYNE_HOR_RESOL 0.16

#define VELODYNE_TOTAL_PACKET_NUMBER 185 // Averagely PACKET_NUMBER is 180 ~ 181. For safty using 185

#pragma pack(push,1)


#define PI 3.14159265358979
#define D2R (PI/180.)
#define R2D (180./PI)

#define BUFF_SIZE 1250



typedef unsigned long DWORD;
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int UINT;
typedef int INT;

typedef struct _VELODYNE_DIST_INTEN_DATA{
    unsigned short distance;
    unsigned char intensity;



}Laser_Data;

typedef struct _VELODYNE_SINGLE_FIRING_DATA{

    unsigned short block_id;
    unsigned short rotation;

    Laser_Data laser_data[32];

}FIRING_DATA;

typedef struct _VELODYNE_ONE_PACKET{

    FIRING_DATA firing_data[12];

    int gps_time_stamp;
    unsigned char status_type;
    unsigned char status_value;

}VELODYNE_DATA;

#pragma pack(pop)
#endif // VELODYNE_PARSER_H
