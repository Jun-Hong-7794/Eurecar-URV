#ifndef GPS_STRUCT_H
#define GPS_STRUCT_H
#pragma pack(1)

struct StrDef_UBX_NAV_PVT // Position, Velocity, Time Solution
{
    unsigned int   iTOW_msec;
    unsigned short year;
    unsigned char  month;
    unsigned char  day;
    unsigned char  hour;
    unsigned char  min;
    unsigned char  sec;
    unsigned char  valid;
    unsigned int   tAcc_nsec;
             int   nano_nsec;
    unsigned char  fixType;
    unsigned char  flags;
    unsigned char  flags2;
    unsigned char  numSV;
             int   lon_deg_em7;
             int   lat_deg_em7;
             int   height_mm;
             int   hMSL_mm;
    unsigned int   hAcc_mm;
    unsigned int   vAcc_mm;
             int   velN_mmps;
             int   velE_mmps;
             int   velD_mmps;
             int   gSpeed_mmps;
             int   headMot_deg_em5;
    unsigned int   sAcc_mmps;
    unsigned int   headAcc_deg_em5;
    unsigned short pDOP_em2;
    unsigned char  reserved2[6];
             int   headVeh_deg_em5;
    unsigned char  reserved3[4];

};

#pragma pack(1)
struct StrDef_UBX_NAV_POSECEF // ECEF Position Solution from Ublox GNSS Module
{
    unsigned int iTOW_msec;
             int ecefX_cm;
             int ecefY_cm;
             int ecefZ_cm;
    unsigned int pAcc_cm;
};
#pragma pack()

struct Gpspoint
{
    long double lat;
    long double lon;
    long double height;
};

struct Ground_Gpspoint
{
    Gpspoint left;
    Gpspoint lefttop;
    Gpspoint righttop;
    Gpspoint right;
};

struct Ground_Bodypoint // calculated body coordinate using current gps position and heading with ground geometry
{
    double dist_left;
    double dist_lefttop;
    double dist_righttop;
    double dist_right;

    double angle_left;
    double angle_lefttop;
    double angle_righttop;
    double angle_right;


};

#endif // GPS_STRUCT_H
