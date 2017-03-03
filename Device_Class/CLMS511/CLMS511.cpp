#include "CLMS511.h"

uint8_t Ascii2Hex(char _ascii)
{
    uint8_t return_val = 0;
    switch(_ascii)
    {
    case 48:
        return_val = 0;
        break;
    case 49:
        return_val = 1;
        break;
    case 50:
        return_val = 2;
        break;
    case 51:
        return_val = 3;
        break;
    case 52:
        return_val = 4;
        break;
    case 53:
        return_val = 5;
        break;
    case 54:
        return_val = 6;
        break;
    case 55:
        return_val = 7;
        break;
    case 56:
        return_val = 8;
        break;
    case 57:
        return_val = 9;
        break;
    case 65:
        return_val = 10;
        break;
    case 66:
        return_val = 11;
        break;
    case 67:
        return_val = 12;
        break;
    case 68:
        return_val = 13;
        break;
    case 69:
        return_val = 14;
        break;
    case 70:
        return_val = 15;
        break;
    case 71:
        return_val = 16;
        break;
    default:
        return_val = 0;
        break;
    }
    return return_val;
}

CLMS511::CLMS511()
{

}

CLMS511::CLMS511(CPCL *_p_pcl)
{
    mpc_pcl = _p_pcl;
}

CLMS511::~CLMS511()
{
    running_command = false;
    this->exit();
}

bool CLMS511::IsLMS511Init()
{
    return lms511_init;
}

bool CLMS511::ConnectLMS511()
{

    client_socket = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP);

    if(client_socket == -1)
    {
        cout << "socket creation failed!" << endl;
        return false;
    }

    memset(&server_addr,0,sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(2111);
    server_addr.sin_addr.s_addr = inet_addr("192.168.0.50");

    if(::connect(client_socket,(struct sockaddr*)&server_addr,sizeof(server_addr)) < 0)
    {
        cout<<"connection failed!"<<endl;
        return false;
    }

    lms_struct_start_measurement lms_start_measurment;
    lms_struct_start_measurement_reply lms_start_measurment_reply;

    write(client_socket,&lms_start_measurment,18);
    read(client_socket,&lms_start_measurment_reply,20);


    lms_struct_device_status lms_device_status;

    char device_status[57];
    memset(&device_status,0,57);

    write(client_socket,&lms_device_status, 11);
    read(client_socket,device_status,57);

    while(!(device_status[11] = 7))
    {
        cout<<"LMS is not ready to measuring!"<<endl;
        return false;
    }

    this->start();

    lms511_init = true;

    return true;
}

void CLMS511::DataRecv()
{
//    mtx_lms.lock();
    lms_struct_poll_telegram lms_poll_telegram;
    char telegram_buff[TELEGRAM_SIZE];

    memset(&telegram_buff,0,TELEGRAM_SIZE);
    write(client_socket,&lms_poll_telegram,17);
    read(client_socket,&telegram_buff,TELEGRAM_SIZE);
    vector<vector<char>> telegrem_buff_parse;

    vector<char> telegrem_buff_parse_element;
    for(int i = 1; i < TELEGRAM_SIZE;i++)
    {
        if(telegram_buff[i] == 0x20)
        {
            telegrem_buff_parse.push_back(telegrem_buff_parse_element);
            telegrem_buff_parse_element.clear();
        }
        else
        {
            telegrem_buff_parse_element.push_back(telegram_buff[i]);
        }
    }
    if((telegrem_buff_parse.size() < 25) || ((telegrem_buff_parse.at(25)).size() < 3) )
        return;

    int lms_data_num = Ascii2Hex((telegrem_buff_parse.at(25)).at(0))*16*16 + Ascii2Hex((telegrem_buff_parse.at(25)).at(1))*16 + Ascii2Hex((telegrem_buff_parse.at(25)).at(2));
    if(telegrem_buff_parse.size() == (unsigned int)(lms_data_num+31))
    {
        vector<vector<double>> lms511_point_list;
        for(int j= 26;j<= (26+lms_data_num-1);j++)
        {
            uint16_t distance = 0;
            memset(&distance,0,2);

            switch((telegrem_buff_parse.at(j)).size())
            {
            case 1:
                distance = Ascii2Hex((telegrem_buff_parse.at(j)).at(0));
                break;
            case 2:
                distance = Ascii2Hex((telegrem_buff_parse.at(j)).at(0))*16+Ascii2Hex((telegrem_buff_parse.at(j)).at(1));
                break;
            case 3:
                distance = Ascii2Hex((telegrem_buff_parse.at(j)).at(0))*16*16 + Ascii2Hex((telegrem_buff_parse.at(j)).at(1))*16+Ascii2Hex((telegrem_buff_parse.at(j)).at(2));
                break;
            case 4:
                distance = Ascii2Hex((telegrem_buff_parse.at(j)).at(0))*16*16*16 + Ascii2Hex((telegrem_buff_parse.at(j)).at(1))*16*16 + Ascii2Hex((telegrem_buff_parse.at(j)).at(2))*16+Ascii2Hex((telegrem_buff_parse.at(j)).at(3));
                break;
            }

            int point_index = j - 26;
            double start_angle = (-5.0)/180.0*3.1415926535;
            double angle_step = (190.0/(lms_data_num - 1))/180.0*3.1415926535;
            double current_angle = start_angle + point_index*angle_step;

            vector<double> lms511_point;
            double lms511_point_x = (double)distance*0.001*cos(current_angle);
            double lms511_point_y = (double)distance*0.001*sin(current_angle);
            lms511_point.push_back(lms511_point_x);
            lms511_point.push_back(lms511_point_y);
            lms511_point_list.push_back(lms511_point);
        }
        emit SignalLMS511UpdatePoints(lms511_point_list);
        parse_complete = true;
    }
    else
    {
        parse_complete = false;
    }
//    mtx_lms.unlock();
}


void CLMS511::run()
{
    while(running_command)
    {
        DataRecv();
        msleep(30);
    }
}
