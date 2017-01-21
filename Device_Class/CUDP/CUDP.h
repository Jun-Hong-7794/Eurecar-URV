#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <iostream>

#define USING_MINE 0x0001
#define USING_DEST 0x0010
#define USING_BOTH 0x0011

class CUDP
{
public:
    CUDP();

private:
    bool fl_socket_init;

private:
    int m_dst_sock;
    int m_my__sock;

    struct sockaddr_in   m_dst_addr;
    struct sockaddr_in   m_my__addr;

public:
    bool Socket_Init(int _mode, char* _dst_ip, int _dst_port, int _time_out_option = 0, char* _my_ip = 0, int _my_port = 0);

    bool Data_Receive(void* _data,int _size);
    bool Data_Send(void* _data,int _size);
};
