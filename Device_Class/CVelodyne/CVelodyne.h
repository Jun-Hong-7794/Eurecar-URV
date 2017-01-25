#ifndef CVELODYNE_H
#define CVELODYNE_H

#include <QThread>
#include <QMutex>
#include <iostream>
#include <QMetaType>
#include <QStandardItemModel>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "CPCL.h"
#include "../CUDP/CUDP.h"

class CVelodyne : public QThread
{
    Q_OBJECT
protected:
    void run();

public:
    CVelodyne();
    ~CVelodyne();

private:
    // PCL Class
    CPCL* mpc_pcl;
    QMutex mtx_pcl;

    bool fl_parser_complete;

    bool fl_pause_status = false;

    QMutex mtx_pcl_class;
    QMutex mtx_parse_state;

    //UDP Class
    CUDP mc_udp;

    int sock;
    unsigned int client_addr_size;

    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;

    bool UDPInitilization();

    // Velodyne
    bool fl_velodyne_init;
    bool fl_velodyne_thread;

    VELODYNE_DATA m_velodyne_data_ary[VELODYNE_TOTAL_PACKET_NUMBER];

    std::vector<double> panel_point_loc_x;
    std::vector<double> panel_point_loc_y;

    double sum_panel_x = 0.0;
    double sum_panel_y = 0.0;
    double mean_panel_x = 0.0;
    double mean_panel_y = 0.0;
    double sum_dist = 0.0;
    double mean_dist = 0.0;

    bool RunVelodyne();
public:
    bool ConnectVelodyne();
    bool SetVelodyneThread(bool _thread_switch);

    bool IsVelodyneConneted();

    // Get panel center x and y location
    std::vector<double> GetPanelCenterLoc();

public:
    //PCL
    CPCL* GetPCL();
    void PCLInitialize();

public slots:
    void SignalRecieveParam(bool ps);

signals:
    void SignalVelodyneParser(bool);
};

#endif // CVELODYNE_H











