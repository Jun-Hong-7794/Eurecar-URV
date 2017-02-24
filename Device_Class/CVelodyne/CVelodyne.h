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

#include "Device_Class/CIMU/CIMU.h"
#include "Device_Class/CGPS/CGPS.h"
#include "Device_Class/CGPS/gps_struct.h"


#define PI 3.14159265358979

#define VELODYNE_MODE_DRIVING 0
#define VELODYNE_MODE_PARKING 1

typedef int VELODYNE_MODE;

class CVelodyne : public QThread
{
    Q_OBJECT
protected:
    void run();

public:
    CVelodyne();
    CVelodyne(CGPS* _p_gps);

    ~CVelodyne();

private:

    // PCL Class
    CPCL* mpc_pcl;
    QMutex mtx_pcl;

    //GPS Class
    CGPS* mpc_gps;
    Ground_Gpspoint m_ground_gps;
    Ground_Bodypoint m_ground_body;
    Gpspoint m_current_gps;
    double m_cur_heading;

    bool fl_gps_init;



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

    double panel_point_x[6];
    double panel_point_y[6];

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

    double lrf_slope_norm_x = 0.0;
    double lrf_slope_norm_y = 0.0;

    double lrf_ransac_line_mean_x = 0;
    double lrf_ransac_line_mean_y = 0;

    double lrf_panel_length = 0.0;

    bool lrf_find_panel = false;

    vector<cv::Point2f> ground_point;



    double velodyne_range = 100.0;
    VELODYNE_MODE velodyne_mode = VELODYNE_MODE_DRIVING;

    bool RunVelodyne();
public:
    bool ConnectVelodyne();
    bool SetVelodyneThread(bool _thread_switch);

    bool IsVelodyneConneted();

    bool IsPanelFound();

    // Get panel center x and y location
    std::vector<double> GetPanelCenterLoc();

    // Get UGV way point x and y
    std::vector<double> GetWaypoint();

    // Get panel find status
    bool GetPanelFindStatus();

    // Get ugv turn direction
    bool GetUGVTurnDirection();

    // Get lrf find panel status
    bool GetLRFPanelFindStatus();

    // Get lrf slope and panel legth
    std::vector<double> GetLRFPanelInfo();

    // Set lrf data
    bool SetLRFDataToPCL(long* _lrf_data,int _num_of_points);
    void SetLRFWaypointToPCL(vector<vector<double>> _waypoint);

    void SetVelodyneRange(double _range);

    void SetVelodyneMode(VELODYNE_MODE _mode);

    //Get panel point
    double* GetPanelPoint_x();
    double* GetPanelPoint_y();

    // Get current_waypoint_index
    int GetCurrentWaypointIndex();

    //calc ground gps points to body coordiante
//    Ground_Bodypoint GetGroundGPS_Body();


    void viewer_update_geofence(pcl::visualization::PCLVisualizer& viewer);

//    // Velodyne with IMU
//    vector<cv::Point2f> CalcGroundBodyPoint_IMU();
//    bool CheckInBoundary_IMU(vector<cv::Point2f> _ground_point, double _obj_x, double _obj_y);

public:
    //PCL
    CPCL* GetPCL();
    void PCLInitialize();

public:
    //GPS
    CGPS* GetGPS();

public slots:
    void SignalRecieveParam(bool ps);

signals:
    void SignalVelodyneParser(bool);

};

typedef struct PanelPointInfo{
    double x;
    double y;
}panel_point_info;



#endif // CVELODYNE_H











