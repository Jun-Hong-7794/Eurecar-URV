#ifndef LMS511_PARSER_H
#define LMS511_PARSER_H

#define TELEGRAM_SIZE 5798
#define TELEGRAM_MEASUREMENT_SIZE 4
#define LMS_POINT_NUM 1141
#include <string>

using namespace std;

#pragma pack(push,1)

typedef struct LMS_STRUCT_LOGIN{
    uint8_t stx = 0x02;
    string command_type = "sMN";
    uint8_t spc1 = ' ';
    string command = "SetAccessMode";
    uint8_t spc2 = ' ';
    int8_t user_level = 3;
    uint8_t spc3 = ' ';
    uint8_t password[4] = {0xF4,0x72,0x47,0x44};
    uint8_t etx = 0x03;

}lms_struct_login;

typedef struct LMS_STRUCT_LOGIN_REPLY{
    uint8_t stx;
    uint8_t command_type[3];
    uint8_t spc1 = ' ';
    uint8_t command[13];
    uint8_t spc2 = ' ';
    bool change_user_level;
    uint8_t etx;

}lms_struct_login_reply;

typedef struct LMS_STRUCT_ACTIVATE_STANBY{
    uint8_t stx = 0x02;
    uint8_t command[14] = {'s','M','N',' ','L','M','C','s','t','a','n','d','b','y'};
    uint8_t etx = 0x03;
}lms_struct_activate_stanby;

typedef struct LMS_STRUCT_ACTIVATE_STANBY_REPLY{
    uint8_t stx;
    uint8_t command[15];
    uint8_t status_code;
    uint8_t etx;
}lms_struct_activate_stanby_reply;


typedef struct LMS_STRUCT_START_MEASUREMENT{
    uint8_t stx = 0x02;
    uint8_t command[16] = {'s','M','N',' ','L','M','C','s','t','a','r','t','m','e','a','s'};
    uint8_t etx = 0x03;
}lms_struct_start_measurement;


typedef struct LMS_STRUCT_START_MEASUREMENT_REPLY{
    uint8_t stx;
    uint8_t command[17];
    uint8_t status_code;
    uint8_t etx;
}lms_struct_start_measurement_reply;

typedef struct LMS_STRUCT_POLL_TELEGRAM{
    uint8_t stx = 0x02;
    uint8_t command[15] = {'s','R','N',' ','L','M','D','s','c','a','n','d','a','t','a'};
    uint8_t etx = 0x03;
}lms_struct_poll_telegram;

typedef struct LMS_READ_DEVICE_STATUS{
    uint8_t stx = 0x02;
    uint8_t command[9] = {'s','R','N',' ','S','T','l','m','s'};
    uint8_t etx = 0x03;
}lms_struct_device_status;

typedef struct LMS_READ_DEVICE_STATUS_REPLY{
    uint8_t stx;
    uint8_t command[9];
    uint8_t spc;
    uint8_t status_code[2];
    uint8_t remanins[43];
    uint8_t etx;
}lms_struct_device_status_reply;




#pragma pack(pop)
#endif // LMS511_PARSER_H
