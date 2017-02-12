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
#include <vector>

#include "CPCL.h"
#include "../CUDP/CUDP.h"

#define PI 3.14159265358979

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

    double waypoint_x = 0.0;
    double waypoint_y = 0.0;

    int matching_point1_index = -1;
    int matching_point2_index = -1;

    double matching_point1_x = 0.0;
    double matching_point1_y = 0.0;

    double matching_point2_x = 0.0;
    double matching_point2_y = 0.0;

    double optimization_direction_left = true; // true means optimized path is left way points

    int current_waypoint_index = 0;

    bool find_panel_point = false;

    bool RunVelodyne();
public:
    bool ConnectVelodyne();
    bool SetVelodyneThread(bool _thread_switch);

    bool IsVelodyneConneted();

    // Get panel center x and y location
    std::vector<double> GetPanelCenterLoc();

    // Get UGV way point x and y
    std::vector<double> GetWaypoint();

    // Get panel find status
    bool GetPanelFindStatus();

    // Get ugv turn direction
    bool GetUGVTurnDirection();

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











