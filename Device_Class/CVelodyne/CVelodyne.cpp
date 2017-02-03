#include "CVelodyne.h"


CVelodyne::CVelodyne(){

    fl_velodyne_init = false;
    fl_parser_complete = false;
    fl_velodyne_thread = false;

    mpc_pcl = new CPCL;
}

CVelodyne::~CVelodyne(){

    SetVelodyneThread(false);
}

void CVelodyne::SignalRecieveParam(bool _velodyne_parse){
    mtx_parse_state.lock();
    {
        fl_pause_status  = _velodyne_parse;
    }
    mtx_parse_state.unlock();
}

CPCL* CVelodyne::GetPCL(){
    return mpc_pcl;
}

void CVelodyne::PCLInitialize(){
    mtx_pcl_class.lock();
    {
        mpc_pcl->init();
    }
    mtx_pcl_class.unlock();
}

bool CVelodyne::UDPInitilization(){

    return mc_udp.Socket_Init(USING_BOTH,(char*)"192.168.0.11",2368,100,(char*)"192.168.0.15",2368);
}

bool CVelodyne::ConnectVelodyne(){

    if(fl_velodyne_init)
        return true;

    if(!UDPInitilization())
        return false;

    mtx_pcl_class.lock();
    {
        mpc_pcl->m_x_data = new double*[VELODYNE_LASERS_NUM];
        mpc_pcl->m_y_data = new double*[VELODYNE_LASERS_NUM];
        mpc_pcl->m_z_data = new double*[VELODYNE_LASERS_NUM];
        mpc_pcl->m_dist_data = new unsigned int*[VELODYNE_LASERS_NUM];

        for (int i = 0; i < VELODYNE_LASERS_NUM; i++){

            mpc_pcl->m_x_data[i] = new double[VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER];
            mpc_pcl->m_y_data[i] = new double[VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER];
            mpc_pcl->m_z_data[i] = new double[VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER];
            mpc_pcl->m_dist_data[i] = new unsigned int[VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER];

            memset(mpc_pcl->m_x_data[i], 0, VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER);
            memset(mpc_pcl->m_y_data[i], 0, VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER);
            memset(mpc_pcl->m_z_data[i], 0, VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER);
            memset(mpc_pcl->m_dist_data[i], 0, VELODYNE_BOLCKS_NUM*VELODYNE_TOTAL_PACKET_NUMBER);
        }

        memset(mpc_pcl->m_velodyne_data_ary,0,sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);
    }
    mtx_pcl_class.unlock();

    fl_velodyne_init = true;

    return true;
}

bool CVelodyne::SetVelodyneThread(bool _thread_switch){

    fl_velodyne_thread = _thread_switch;

    if(fl_velodyne_thread){

        if(!ConnectVelodyne())
            return false;

        this->start();

        return true;
    }
    else{// Wait for Thread Off
        while(this->isRunning());
    }
    return true;
}

bool CVelodyne::IsVelodyneConneted(){
    return fl_velodyne_thread;
}

bool CVelodyne::RunVelodyne(){

    if(!fl_velodyne_init){
        std::cout << "Velodyne Init First!" << std::endl;
        return false;
    }

    UINT count = 0;
    WORD rotation = 0;
    double prev_deg = 0.0;
    double deg = 0.0;


    VELODYNE_DATA buffer[VELODYNE_TOTAL_PACKET_NUMBER];
    memset(buffer,0,sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

    size_t point_index = 0;
    size_t panel_point_index = 0;



    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);




    while(fl_velodyne_thread){

        if(!mc_udp.Data_Receive((char *)&buffer[count],VELODYNE_DATA_SIZE)){
            cout << "Fail to Receive@@@" << endl;
            continue;
        }

        rotation = (buffer[count].firing_data[0].rotation);

        deg = (double)(rotation / 100.0);

        if (deg < prev_deg){


            sum_panel_x = 0.0;
            sum_panel_y = 0.0;
            sum_dist = 0.0;
            mean_panel_x = 0.0;
            mean_panel_y = 0.0;
            mean_dist = 0.0;

            mtx_pcl_class.lock();
            {
                memcpy(mpc_pcl->m_velodyne_data_ary, buffer,
                    sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                mpc_pcl->Set_Velodyne_Data(mpc_pcl->m_x_data,mpc_pcl->m_y_data,mpc_pcl->m_z_data);

                mpc_pcl->cloud->points.resize(VELODYNE_LASERS_NUM*VELODYNE_TOTAL_PACKET_NUMBER*VELODYNE_BOLCKS_NUM);

                point_index = 0;
                panel_point_index = 0;


                double minimum_z = -1.22;
                double tolerence = 0.5;

                pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter(true);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ei_result (new pcl::PointCloud<pcl::PointXYZRGBA>);




                for(int k = 0;k < VELODYNE_LASERS_NUM;k++){
                    for(int i=0; i< VELODYNE_TOTAL_PACKET_NUMBER;i++){
                        for(int j=0; j < VELODYNE_BOLCKS_NUM;j++){
                            mpc_pcl->cloud->points[point_index].x = mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                            mpc_pcl->cloud->points[point_index].y = mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                            mpc_pcl->cloud->points[point_index].z = mpc_pcl->m_z_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;


                            if((firing_vertical_angle[k] == 0)
                                    && (mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] > 700)
                                    && (mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] < 2000)
                                    && (mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)] < 0 )){

                                sum_panel_x += mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                                sum_panel_y += mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                                sum_dist += mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

                                panel_point_index++;

                                mpc_pcl->cloud->points[point_index].r = 255;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].g = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].b = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
                            }
                            else{
                                mpc_pcl->cloud->points[point_index].r = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].g = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].b = 255;// *(1024 * rand () / (RAND_MAX + 1.0f));
                            }

                            if(std::abs((mpc_pcl->cloud->points[point_index].z - minimum_z)) > tolerence)
                            {
                                (*inliers).indices.push_back(point_index);
                                /*
                                mpc_pcl->cloud->points[point_index].r = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].g = 255;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].b = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));*/
                            }

                            point_index++;
                        }
                    }
                }

                mean_panel_x = sum_panel_x/(double)(panel_point_index+1.0);
                mean_panel_y = sum_panel_y/(double)(panel_point_index+1.0);
                mean_dist = sum_dist/(double)(panel_point_index+1.0);



                eifilter.setInputCloud(mpc_pcl->cloud);
                eifilter.setIndices(inliers);
                eifilter.filter(*ei_result);

                (*inliers).indices.clear();

//                mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");
                mpc_pcl->viewer->updatePointCloud(ei_result,"cloud");




            }
            mtx_pcl_class.unlock();

            fl_parser_complete = true;

            emit SignalVelodyneParser(fl_parser_complete);

            memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

            count = 0;
        }

        prev_deg = deg;

        if (count >= VELODYNE_TOTAL_PACKET_NUMBER){
            count = 0;
        }

        count++;
        QThread::usleep(10);
    }

    return true;
}

std::vector<double> CVelodyne::GetPanelCenterLoc()
{

    std::vector<double> mean_panel_loc;

    mean_panel_loc.push_back(mean_panel_x);
    mean_panel_loc.push_back(mean_panel_y);
    mean_panel_loc.push_back(mean_dist);

    return mean_panel_loc;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CVelodyne::run(){

    RunVelodyne();
}



















