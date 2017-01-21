#include "CUDP.h"

CUDP::CUDP(){
    fl_socket_init = false;
}

bool CUDP::Socket_Init(int _mode, char* _dst_ip, int _dst_port, int _time_out_option, char* _my_ip, int _my_port){

    m_my__sock  = socket( PF_INET, SOCK_DGRAM, 0);

    if(m_my__sock == -1){
        std::cout << "Fail to Create Socket" << std::endl;
        return false;
    }

    int reuse = 1;
    if ( setsockopt(m_my__sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse) ) < 0 ) {
        std::cout << "Fail to SO_REUSEADDR failed" << std::endl;
        return false;
    }

    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = _time_out_option * 1000;

    if(_time_out_option != 0)
        setsockopt(m_my__sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv));


    if(_mode & USING_DEST){
        m_dst_sock  = socket( PF_INET, SOCK_DGRAM, 0);

        if(m_dst_sock == -1){
            std::cout << "Fail to Create Socket" << std::endl;
            return false;
        }

        memset( &m_dst_addr, 0, sizeof( m_dst_addr));
        m_dst_addr.sin_family     = AF_INET;
        m_dst_addr.sin_port       = htons( _dst_port);
        m_dst_addr.sin_addr.s_addr= inet_addr(_dst_ip);
    }

    if(_mode & USING_MINE){

        memset( &m_my__addr, 0, sizeof( m_my__addr));
        m_my__addr.sin_family     = AF_INET;
        m_my__addr.sin_port       = htons( _my_port);
        m_my__addr.sin_addr.s_addr= inet_addr(_my_ip);

        if(bind( m_my__sock, (struct sockaddr*)&m_my__addr, sizeof( m_my__addr) ) == -1 ){
           std::cout << "Fail to bind() in Destination Socket" << std::endl;
           return false;
        }
    }

    return true;
}

bool CUDP::Data_Receive(void* _data,int _size){

    int recvDatalen = 0;
    socklen_t dst_addr_len = sizeof(m_dst_addr);

    if ((recvDatalen = recvfrom(m_my__sock, (char *)_data, _size, 0,
                                (struct sockaddr *) &m_dst_addr, &dst_addr_len)) < 0){
         return false;
    }
    else{
         return true;
    }
}

bool CUDP::Data_Send(void* _data,int _size){

    int sendDatalen = 0;
    socklen_t dst_addr_len = sizeof(m_dst_addr);

    if ((sendDatalen = sendto(m_my__sock, (char *)_data, _size, 0, (struct sockaddr *) &m_dst_addr,
                              dst_addr_len)) != _size){
        printf("send err, send Data size : %d\n",sendDatalen);
    }

    return true;
}
