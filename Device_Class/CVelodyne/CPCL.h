#ifndef CPCL_H
#define CPCL_H

#include "velodyne_parser.h"

//-------------------------------------------------
// VTK - pck header
//-------------------------------------------------
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
//for projection
#include <pcl/filters/project_inliers.h>

#include <vtkRenderWindow.h>
#include <vtkAutoInit.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

const double firing_vertical_angle[32] = {
    -30.67, -9.33, -29.33, -8.00, -28.00,
    -6.66, -26.66, -5.33, -25.33, -4.00,
    -24.00, -2.67, -22.67, -1.33, -21.33,
    0.00, -20.00, 1.33, -18.67, 2.67,
    -17.33, 4.00, -16.00, 5.33, -14.67,
    6.67, -13.33, 8.00, -12.00, 9.33,
    -10.67, 10.67
};


const double prior_panel_points[6][2] = {{0,0}, {1.0,0},{1.0,0.25},{1.0,0.75},{0,0.75},{0,0.25}};
//const double prior_way_points[4][2] = {{2.0,1.75},{-1.0,1.75},{-1.0,-1.0},{0.5,-1.0}};
const double prior_way_points_left[3][2] = {{-1.0,1.75},{-1.0,-1.0},{0.5,-1.0}};
const double prior_way_points_right[3][2] = {{2.0,1.75},{2.0,-1.0},{0.5,-1.0}};
const double prior_way_point_lrf_left[5][3] = {{0.5,1.75,PI},{-1.0,1.75,1.5*PI},{-1.0,0.375,1.5*PI},{-1.0,-1.0,0},{0.5,-1.0,0}};
const double prior_way_point_lrf_right[4][3] = {{2.0,1.75,0.5*PI},{2.0,1.75,0.5*PI},{2.0,1.75,0.5*PI},{2.0,1.75,0.5*PI}};
const double driving_waypoint_x_margin = 1.2;
const double driving_waypoint_y_margin = 0.5;
const double velodyne_range_driving = 1.5;
const double velodyne_range_parking = 3.0;

class CPCL
{
public:
    CPCL(){

    }
    void init()
    {
        // Setup the cloud pointer
        cloud.reset(new PointCloudT);
        waypoint_cloud.reset(new PointCloudT);
        panelpoint_cloud.reset(new PointCloudT);
        lrf_cloud.reset(new PointCloudT);
        // PCL display setting
        viewer.reset (new pcl::visualization::PCLVisualizer ("viewer" , false));

    }

    bool velodyne_loop = false;
    VELODYNE_DATA m_velodyne_data_ary[VELODYNE_TOTAL_PACKET_NUMBER];//360 deg Data

    void Set_Velodyne_Data(double **_x_ary, double **_y_ary, double **_z_ary){
        for (int k = 0; k < VELODYNE_LASERS_NUM; k++){
            for (int i = 0; i < VELODYNE_TOTAL_PACKET_NUMBER; i++){
                for (int j = 0; j < VELODYNE_BOLCKS_NUM; j++){

                    double z_projected = 0.0;
                    double xy_deg = 0.0;

                    unsigned int distance = (m_velodyne_data_ary[i].firing_data[j].laser_data[k].distance) * 2;//mm
                    unsigned short rotation = (m_velodyne_data_ary[i].firing_data[j].rotation);
                    if(distance != 0)
                    xy_deg = rotation / 100.0;
                    z_projected = distance * cos(D2R*(firing_vertical_angle[k]));
                    if(z_projected != 0)
                    m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] = distance;

                    _x_ary[k][(j + i*VELODYNE_BOLCKS_NUM)] =
                        z_projected*sin(D2R*xy_deg);

                    _y_ary[k][(j + i*VELODYNE_BOLCKS_NUM)] =
                        z_projected*cos(D2R*xy_deg);

                    _z_ary[k][(j + i*VELODYNE_BOLCKS_NUM)] =
                        distance * sin(D2R*(firing_vertical_angle[k]));
                }
            }
        }
    }

    double **m_x_data = NULL;
    double **m_y_data = NULL;
    double **m_z_data = NULL;
    unsigned int **m_dist_data;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud;
    PointCloudT::Ptr waypoint_cloud;
    PointCloudT::Ptr panelpoint_cloud;
    PointCloudT::Ptr lrf_cloud;
};

#endif // CPCL_H
