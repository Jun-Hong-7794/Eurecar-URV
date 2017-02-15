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

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);

    pcl::PointIndices::Ptr tmp_inlier(new pcl::PointIndices);
    pcl::PointIndices::Ptr max_points_clustering_indices(new pcl::PointIndices);

    pcl::PointIndices::Ptr tmp_inlier1(new pcl::PointIndices);
    pcl::PointIndices::Ptr max_points_clustering_indices1(new pcl::PointIndices);

    pcl::PointIndices::Ptr disp_indices(new pcl::PointIndices);

    double clustering_tolerence = 0.1;
    double clustering_count_tolerence = 50;
    double waypoint_converged_margin = 0.15;

    while(fl_velodyne_thread)
    {

        if(!mc_udp.Data_Receive((char *)&buffer[count],VELODYNE_DATA_SIZE)){
            cout << "Fail to Receive@@@" << endl;
            continue;
        }

        rotation = (buffer[count].firing_data[0].rotation);

        deg = (double)(rotation / 100.0);

        if (deg < prev_deg){
            mtx_pcl_class.lock();


            sum_panel_x = 0.0;
            sum_panel_y = 0.0;
            sum_dist = 0.0;
            mean_panel_x = 0.0;
            mean_panel_y = 0.0;
            mean_dist = 0.0;

            (*inliers).indices.clear();
            (*inliers2).indices.clear();


            memcpy(mpc_pcl->m_velodyne_data_ary, buffer,
                sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

            mpc_pcl->Set_Velodyne_Data(mpc_pcl->m_x_data,mpc_pcl->m_y_data,mpc_pcl->m_z_data);

            mpc_pcl->cloud->points.resize(VELODYNE_LASERS_NUM*VELODYNE_TOTAL_PACKET_NUMBER*VELODYNE_BOLCKS_NUM);
            mpc_pcl->waypoint_cloud->points.resize(3);
            mpc_pcl->panelpoint_cloud->points.resize(6);

            point_index = 0;
            panel_point_index = 0;


            double minimum_z = 0.;
            double tolerence = 0.;


            pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter(true);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ei_result (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter2(true);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ei_result2 (new pcl::PointCloud<pcl::PointXYZRGBA>);


            pcl::ExtractIndices<pcl::PointXYZRGBA> max_clustering_filter(true);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr max_clustering_result (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::ExtractIndices<pcl::PointXYZRGBA> max_clustering_filter1(true);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr max_clustering_result1 (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::vector<int> inliers_tmp;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::vector<int> inliers_tmp1;

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ransac_result_lrf (new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::vector<int> inliers_lrf;


            (*disp_indices).indices.clear();


            for(int k = 0;k < VELODYNE_LASERS_NUM;k++){
                for(int i=0; i< VELODYNE_TOTAL_PACKET_NUMBER;i++){
                    for(int j=0; j < VELODYNE_BOLCKS_NUM;j++){
                        mpc_pcl->cloud->points[point_index].x = mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                        mpc_pcl->cloud->points[point_index].y = mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                        mpc_pcl->cloud->points[point_index].z = mpc_pcl->m_z_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

                        if(mpc_pcl->cloud->points[point_index].z < minimum_z)
                        {
                            minimum_z = mpc_pcl->cloud->points[point_index].z;

                        }
                        else{
                            mpc_pcl->cloud->points[point_index].r = 0;
                            mpc_pcl->cloud->points[point_index].g = 0;
                            mpc_pcl->cloud->points[point_index].b = 255;
                        }


                        point_index++;
                    }
                }
            }
            point_index = 0;

            double past_point_x = .0;
            double past_point_y = .0;
            double past_point_z = .0;
            double adjacent_dist = .0;

            double past_point_x2 = .0;
            double past_point_y2 = .0;
            double past_point_z2 = .0;
            double adjacent_dist2 = .0;


            int clustering_index = 0;
            int clustering_index2 = 0;
            int clustering_member_count = 0;
            int clustering_member_count2 = 0;

            int max_clustering_member_count = 0;
            int max_clustering_member_count1 = 0;



            for(int k = 0;k < VELODYNE_LASERS_NUM;k++){
                for(int i=0; i< VELODYNE_TOTAL_PACKET_NUMBER;i++){
                    for(int j=0; j < VELODYNE_BOLCKS_NUM;j++){
                        mpc_pcl->cloud->points[point_index].x = mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                        mpc_pcl->cloud->points[point_index].y = mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                        mpc_pcl->cloud->points[point_index].z = mpc_pcl->m_z_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

                        if((std::abs((mpc_pcl->cloud->points[point_index].z - minimum_z)) > tolerence) && ((mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] < velodyne_range*1000.0)) && mpc_pcl->cloud->points[point_index].x < 0)
                        {
                            if((firing_vertical_angle[k] == 0.0))
                            {
                                adjacent_dist = std::sqrt(std::pow(mpc_pcl->cloud->points[point_index].x - past_point_x,2) + std::pow(mpc_pcl->cloud->points[point_index].y - past_point_y,2) + std::pow(mpc_pcl->cloud->points[point_index].z - past_point_z,2)) ;

                                past_point_x = mpc_pcl->cloud->points[point_index].x;
                                past_point_y = mpc_pcl->cloud->points[point_index].y;
                                past_point_z = mpc_pcl->cloud->points[point_index].z;

                                if(adjacent_dist > 0.0)
                                {

                                    sum_panel_x += mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                                    sum_panel_y += mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                                    sum_dist += mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

                                    panel_point_index++;


                                    if(adjacent_dist < clustering_tolerence)
                                    {
                                        (*inliers).indices.push_back(point_index);

                                        (*tmp_inlier).indices.push_back(point_index);
                                        clustering_member_count++;
                                        if(max_clustering_member_count < clustering_member_count)
                                        {
                                            max_clustering_member_count = clustering_member_count;
                                            (*max_points_clustering_indices).indices = (*tmp_inlier).indices;
                                        }

                                        mpc_pcl->cloud->points[point_index].r = 255;
                                        mpc_pcl->cloud->points[point_index].g = 0;
                                        mpc_pcl->cloud->points[point_index].b = 0;
                                    }
                                    else
                                    {
                                        if(clustering_member_count < clustering_count_tolerence)
                                        {
                                            (*inliers).indices.erase((*inliers).indices.end()-clustering_member_count, (*inliers).indices.end());
                                        }

                                        (*tmp_inlier).indices.clear();

                                        clustering_member_count = 0;
                                        clustering_index++;
                                    }

                                }

                            }
                            else if((firing_vertical_angle[k] == 10.67))
                            {
                                adjacent_dist2 = std::sqrt(std::pow(mpc_pcl->cloud->points[point_index].x - past_point_x2,2) + std::pow(mpc_pcl->cloud->points[point_index].y - past_point_y2,2) + std::pow(mpc_pcl->cloud->points[point_index].z - past_point_z2,2)) ;

                                past_point_x2 = mpc_pcl->cloud->points[point_index].x;
                                past_point_y2 = mpc_pcl->cloud->points[point_index].y;
                                past_point_z2 = mpc_pcl->cloud->points[point_index].z;

                                if(adjacent_dist2 >0.)
                                {
                                    if(adjacent_dist2 < clustering_tolerence)
                                    {
                                        (*inliers2).indices.push_back(point_index);
                                        (*tmp_inlier1).indices.push_back(point_index);
                                        clustering_member_count2++;

                                        if(max_clustering_member_count1 < clustering_member_count2)
                                        {
                                            max_clustering_member_count1 = clustering_member_count2;
                                            (*max_points_clustering_indices1 ).indices = (*tmp_inlier1).indices;
                                        }
                                        mpc_pcl->cloud->points[point_index].r = cv::saturate_cast<uchar>(255 - clustering_index2*10);
                                        mpc_pcl->cloud->points[point_index].g = cv::saturate_cast<uchar>(128 + clustering_index2*10);
                                        mpc_pcl->cloud->points[point_index].b = cv::saturate_cast<uchar>(clustering_index2*10);
                                    }
                                    else
                                    {
                                        if(clustering_member_count2 < clustering_count_tolerence)
                                        {
                                            (*inliers2).indices.erase((*inliers2).indices.end()-clustering_member_count2, (*inliers2).indices.end());
                                        }

                                        (*tmp_inlier1).indices.clear();

                                        clustering_member_count2 = 0;
                                        clustering_index2++;
                                    }

                                }

                            }
                            else
                            {
                                mpc_pcl->cloud->points[point_index].r = 0;
                                mpc_pcl->cloud->points[point_index].g = 0;
                                mpc_pcl->cloud->points[point_index].b = 255;
                            }
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

            eifilter2.setInputCloud(mpc_pcl->cloud);
            eifilter2.setIndices(inliers2);
            eifilter2.filter(*ei_result2);

            max_clustering_filter.setInputCloud(mpc_pcl->cloud);
            max_clustering_filter.setIndices(max_points_clustering_indices);
            max_clustering_filter.filter(*max_clustering_result);

            max_clustering_filter1.setInputCloud(mpc_pcl->cloud);
            max_clustering_filter1.setIndices(max_points_clustering_indices1);
            max_clustering_filter1.filter(*max_clustering_result1);

            if(velodyne_mode == VELODYNE_MODE_DRIVING)
            {
                waypoint_x = mean_panel_x + driving_waypoint_x_margin;
                waypoint_y = mean_panel_y + driving_waypoint_y_margin;

                mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");

                fl_parser_complete = true;

                emit SignalVelodyneParser(fl_parser_complete);

                memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                count = 0;

                mtx_pcl_class.unlock();

                continue;
            }

            if(mpc_pcl->lrf_cloud->points.size()>=2)
            {


                Eigen::VectorXf coeff_lrf;
                pcl::SampleConsensusModelLine<pcl::PointXYZRGBA>::Ptr model_lrf(new pcl::SampleConsensusModelLine<pcl::PointXYZRGBA> (mpc_pcl->lrf_cloud));
                pcl::RandomSampleConsensus<pcl::PointXYZRGBA> sac_lrf (model_lrf,0.03);

                sac_lrf.computeModel();
                sac_lrf.getInliers(inliers_lrf);
                sac_lrf.getModelCoefficients(coeff_lrf);


                pcl::copyPointCloud(*mpc_pcl->lrf_cloud,inliers_lrf,*ransac_result_lrf);

                double lrf_ransac_line_sum_x = 0;
                double lrf_ransac_line_sum_y = 0;


                // Calculate ransac mean
                for(int i = 0; i < ransac_result_lrf->points.size();i++)
                {
                    lrf_ransac_line_sum_x += ransac_result_lrf->points[i].x;
                    lrf_ransac_line_sum_y += ransac_result_lrf->points[i].y;
                }
                lrf_ransac_line_mean_x = lrf_ransac_line_sum_x/(double)ransac_result_lrf->points.size();
                lrf_ransac_line_mean_y = lrf_ransac_line_sum_y/(double)ransac_result_lrf->points.size();


                // Calculate maximum distance

                double lrf_maximum_dist_from_ransac_mean = 0;
                for(int i = 0; i < ransac_result_lrf->points.size();i++)
                {
                    double lrf_dist_from_ransac_mean = sqrt((lrf_ransac_line_mean_x - ransac_result_lrf->points[i].x)*(lrf_ransac_line_mean_x - ransac_result_lrf->points[i].x) + (lrf_ransac_line_mean_y - ransac_result_lrf->points[i].y)*(lrf_ransac_line_mean_y - ransac_result_lrf->points[i].y));
                    if (lrf_maximum_dist_from_ransac_mean < lrf_dist_from_ransac_mean)
                    {
                        lrf_maximum_dist_from_ransac_mean = lrf_dist_from_ransac_mean;
                    }
                }


                lrf_panel_length = 2*lrf_maximum_dist_from_ransac_mean;
                double lrf_panel_dist =0.;
                double lrf_panel_slope =0.;

                lrf_panel_dist = sqrt(lrf_ransac_line_mean_x*lrf_ransac_line_mean_x + lrf_ransac_line_mean_y*lrf_ransac_line_mean_y);

                if( coeff_lrf.rows() != 6)
                {
                    mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");

                    lrf_find_panel = false;
                    fl_parser_complete = true;

                    emit SignalVelodyneParser(fl_parser_complete);

                    memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                    count = 0;

                    mtx_pcl_class.unlock();

                    continue;
                }
                else
                {
                    lrf_find_panel = true;
                }

                if(coeff_lrf[3] == 0)
                {
                    mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");

                    fl_parser_complete = true;

                    emit SignalVelodyneParser(fl_parser_complete);

                    memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                    count = 0;

                    mtx_pcl_class.unlock();

                    continue;
                }
                lrf_panel_slope = coeff_lrf[4]/coeff_lrf[3];

                lrf_slope_norm_x = coeff_lrf[3];
                lrf_slope_norm_y = coeff_lrf[4];

                mpc_pcl->viewer->updatePointCloud(ransac_result_lrf,"lrf_cloud");


            }
            else
            {
                lrf_find_panel = false;
            }

            if((max_clustering_result->points.size() < 2) || (max_clustering_result1->points.size() < 2))
            {
                mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");
                find_panel_point = false;
            }
            else
            {
                Eigen::VectorXf coeff;
                pcl::SampleConsensusModelLine<pcl::PointXYZRGBA>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZRGBA> (max_clustering_result));
                pcl::RandomSampleConsensus<pcl::PointXYZRGBA> sac (model,0.03);

                Eigen::VectorXf coeff1;
                pcl::SampleConsensusModelLine<pcl::PointXYZRGBA>::Ptr model1(new pcl::SampleConsensusModelLine<pcl::PointXYZRGBA> (max_clustering_result1));
                pcl::RandomSampleConsensus<pcl::PointXYZRGBA> sac1 (model1,0.03);

                sac.computeModel();
                sac.getInliers(inliers_tmp);
                sac.getModelCoefficients(coeff);


                sac1.computeModel();
                sac1.getInliers(inliers_tmp1);
                sac1.getModelCoefficients(coeff1);

                pcl::copyPointCloud(*max_clustering_result,inliers_tmp,*final);
                pcl::copyPointCloud(*max_clustering_result1,inliers_tmp1,*final1);


                double ransac_line_mean_x = 0;
                double ransac_line_mean_y = 0;
                double ransac_line_sum_x = 0;
                double ransac_line_sum_y = 0;

                double ransac_line1_mean_x = 0;
                double ransac_line1_mean_y = 0;
                double ransac_line1_sum_x = 0;
                double ransac_line1_sum_y = 0;


                if(final->points.size() == 0 || final1->points.size() == 0)
                {
                    mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");

                    fl_parser_complete = true;

                    emit SignalVelodyneParser(fl_parser_complete);

                    memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                    count = 0;

                    mtx_pcl_class.unlock();


                    continue;
                }

                // Calculate ransac mean
                for(int i = 0; i < final->points.size();i++)
                {
                    ransac_line_sum_x += final->points[i].x;
                    ransac_line_sum_y += final->points[i].y;
                }
                ransac_line_mean_x = ransac_line_sum_x/(double)final->points.size();
                ransac_line_mean_y = ransac_line_sum_y/(double)final->points.size();


                // Calculate maximum distance

                double maximum_dist_from_ransac_mean = 0;
                for(int i = 0; i < final->points.size();i++)
                {
                    double dist_from_ransac_mean = sqrt((ransac_line_mean_x - final->points[i].x)*(ransac_line_mean_x - final->points[i].x) + (ransac_line_mean_y - final->points[i].y)*(ransac_line_mean_y - final->points[i].y));
                    if (maximum_dist_from_ransac_mean < dist_from_ransac_mean)
                    {
                        maximum_dist_from_ransac_mean = dist_from_ransac_mean;
                    }
                }

                for(int i = 0; i < final1->points.size();i++)
                {
                    ransac_line1_sum_x += final1->points[i].x;
                    ransac_line1_sum_y += final1->points[i].y;
                }
                ransac_line1_mean_x = ransac_line1_sum_x/(double)final1->points.size();
                ransac_line1_mean_y = ransac_line1_sum_y/(double)final1->points.size();



                // Calculate maximum distance

                double maximum_dist1_from_ransac_mean = 0;
                for(int i = 0; i < final1->points.size();i++)
                {
                    double dist_from1_ransac_mean = sqrt((ransac_line1_mean_x - final1->points[i].x)*(ransac_line1_mean_x - final1->points[i].x) + (ransac_line1_mean_y - final1->points[i].y)*(ransac_line1_mean_y - final1->points[i].y));
                    if (maximum_dist1_from_ransac_mean < dist_from1_ransac_mean)
                    {
                        maximum_dist1_from_ransac_mean = dist_from1_ransac_mean;
                    }
                }

                if( coeff.rows() != 6 || coeff1.rows() != 6)
                {
                    mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");

                    fl_parser_complete = true;

                    emit SignalVelodyneParser(fl_parser_complete);

                    memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                    count = 0;

                    mtx_pcl_class.unlock();


                    continue;
                }

                if ((coeff[3] == 0) || (coeff1[3] == 0))
                {

                    mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");

                    fl_parser_complete = true;

                    emit SignalVelodyneParser(fl_parser_complete);

                    memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                    count = 0;

                    mtx_pcl_class.unlock();


                    continue;
                }

                double b = ransac_line_mean_y - (coeff[4]/coeff[3])*ransac_line_mean_x;
                double b1 = ransac_line1_mean_y - (coeff1[4]/coeff1[3])*ransac_line1_mean_x;

                if (((2.0*maximum_dist_from_ransac_mean) > 0.8) && ((2.0*maximum_dist_from_ransac_mean) < 1.2)) // front or back
                {
                    find_panel_point = true;
                    if( (coeff[3]*coeff1[3] + coeff[4]*coeff1[4]) > 0.5) // 0ch and 10.67 ch is parrell
                    {

                        double line_distance = abs(-(coeff[4]/coeff[3])*ransac_line1_mean_x + ransac_line1_mean_y - b)/sqrt((coeff[4]/coeff[3])*(coeff[4]/coeff[3]) + 1);

                        if(line_distance > 0.2) // front side
                        {
                            if (coeff[3] != 0)
                            {
                                if ( (coeff[4]/coeff[3]) < 0 )
                                {
                                    matching_point1_index = 2;
                                    matching_point1_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);
                                    matching_point2_index = 5;
                                    matching_point2_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    double matching_point_dir_x = matching_point2_x - matching_point1_x;
                                    double matching_point_dir_y = matching_point2_y - matching_point1_y;

                                    double matching_point_dir_norm_x = matching_point_dir_x/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);
                                    double matching_point_dir_norm_y = matching_point_dir_y/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);

                                    double matching_point_dir_angle = atan2(matching_point_dir_norm_y,matching_point_dir_norm_x);

                                    double prior_point_dir_x = prior_panel_points[matching_point2_index][0] - prior_panel_points[matching_point1_index][0];
                                    double prior_point_dir_y = prior_panel_points[matching_point2_index][1] - prior_panel_points[matching_point1_index][1];

                                    double prior_point_dir_norm_x = prior_point_dir_x/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);
                                    double prior_point_dir_norm_y = prior_point_dir_y/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);

                                    double prior_point_dir_angle = atan2(prior_point_dir_norm_y,prior_point_dir_norm_x);

                                    double transform_delta_angle = matching_point_dir_angle - prior_point_dir_angle;

                                    double transform_delta_x_for_prior = -prior_panel_points[matching_point1_index][0];
                                    double transform_delta_y_for_prior = -prior_panel_points[matching_point1_index][1];

                                    double transform_delta_x_for_matching = matching_point1_x;
                                    double transform_delta_y_for_matching = matching_point1_y;

                                    for(int i = 0;i<mpc_pcl->panelpoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_panel_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_panel_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_panel_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_panel_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        mpc_pcl->panelpoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->panelpoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->panelpoint_cloud->points[i].z = 0;
                                        mpc_pcl->panelpoint_cloud->points[i].r = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].g = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].b = 255;
                                    }

                                    // Find optimized waypoints----------
                                    double panel_front_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_front_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_back_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (0.75 + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_back_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (0.75 + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_yaw_direction_x = panel_back_center_x - panel_front_center_x;
                                    double panel_yaw_direction_y = panel_back_center_y - panel_front_center_y;

                                    double prior_way_points[3][2];

                                    if( ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) > 0) &&  ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) < PI)) // right is the fastest
                                    {
                                        optimization_direction_left = false;
                                        copy(&prior_way_points_right[0][0],&prior_way_points_right[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    else
                                    {
                                        optimization_direction_left = true;
                                        copy(&prior_way_points_left[0][0],&prior_way_points_left[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    // ----------------------------------

                                    vector<double> panel_center_to_ugv;

                                    panel_center_to_ugv.push_back(-panel_front_center_x);
                                    panel_center_to_ugv.push_back(-panel_front_center_y);

                                    double angle_from_panel_center;
                                    if(atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0)) < 0)
                                    {
                                        angle_from_panel_center = 2*PI+atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }
                                    else
                                    {
                                        angle_from_panel_center = atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }

                                    for(int i = 0;i<mpc_pcl->waypoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_way_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_way_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_way_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_way_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        double panel_center_to_waypoint_x,panel_center_to_waypoint_y;

                                        panel_center_to_waypoint_x = transform_result_x - panel_front_center_x;
                                        panel_center_to_waypoint_y = transform_result_y - panel_front_center_y;


                                        double outer_product_for_wayupdate = panel_center_to_ugv.at(0)*panel_center_to_waypoint_y - panel_center_to_ugv.at(1)*panel_center_to_waypoint_x;
                                        if(optimization_direction_left)
                                        {
                                            if(outer_product_for_wayupdate < 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }
                                        else
                                        {
                                            if(outer_product_for_wayupdate > 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }


                                        mpc_pcl->waypoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->waypoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->waypoint_cloud->points[i].z = 0;
                                        mpc_pcl->waypoint_cloud->points[i].r = 0;
                                        mpc_pcl->waypoint_cloud->points[i].g = 255;
                                        mpc_pcl->waypoint_cloud->points[i].b = 255;
                                    }
                                    if(current_waypoint_index == 3)
                                    {
                                        current_waypoint_index = 0;
                                    }

                                    if((current_waypoint_index != 2) && (sqrt(mpc_pcl->waypoint_cloud->points[current_waypoint_index].x*mpc_pcl->waypoint_cloud->points[current_waypoint_index].x + mpc_pcl->waypoint_cloud->points[current_waypoint_index].y*mpc_pcl->waypoint_cloud->points[current_waypoint_index].y) < waypoint_converged_margin))
                                    {
                                        current_waypoint_index += 1;
                                    }

                                    double angle_between_panel_yaw_vector = acos((panel_center_to_ugv.at(0)*(-panel_yaw_direction_x)+panel_center_to_ugv.at(1)*(-panel_yaw_direction_y))/(sqrt(panel_center_to_ugv.at(0)*panel_center_to_ugv.at(0) + panel_center_to_ugv.at(1)*panel_center_to_ugv.at(1))*sqrt(panel_yaw_direction_x*panel_yaw_direction_x+panel_yaw_direction_y*panel_yaw_direction_y)));

                                    if(angle_between_panel_yaw_vector < 10.0/180.0*PI)
                                    {
                                        current_waypoint_index = 2;
                                    }

                                    pcl::PointXYZRGBA origin_pt;
                                    origin_pt.x = 0;
                                    origin_pt.y = 0;
                                    origin_pt.z = 0;

                                    waypoint_x = mpc_pcl->waypoint_cloud->points[current_waypoint_index].x;
                                    waypoint_y = mpc_pcl->waypoint_cloud->points[current_waypoint_index].y;

                                }
                                else
                                {
                                    matching_point1_index = 2;
                                    matching_point1_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);
                                    matching_point2_index = 5;
                                    matching_point2_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    double matching_point_dir_x = matching_point2_x - matching_point1_x;
                                    double matching_point_dir_y = matching_point2_y - matching_point1_y;

                                    double matching_point_dir_norm_x = matching_point_dir_x/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);
                                    double matching_point_dir_norm_y = matching_point_dir_y/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);

                                    double matching_point_dir_angle = atan2(matching_point_dir_norm_y,matching_point_dir_norm_x);

                                    double prior_point_dir_x = prior_panel_points[matching_point2_index][0] - prior_panel_points[matching_point1_index][0];
                                    double prior_point_dir_y = prior_panel_points[matching_point2_index][1] - prior_panel_points[matching_point1_index][1];

                                    double prior_point_dir_norm_x = prior_point_dir_x/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);
                                    double prior_point_dir_norm_y = prior_point_dir_y/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);

                                    double prior_point_dir_angle = atan2(prior_point_dir_norm_y,prior_point_dir_norm_x);

                                    double transform_delta_angle = matching_point_dir_angle - prior_point_dir_angle;

                                    double transform_delta_x_for_prior = -prior_panel_points[matching_point1_index][0];
                                    double transform_delta_y_for_prior = -prior_panel_points[matching_point1_index][1];

                                    double transform_delta_x_for_matching = matching_point1_x;
                                    double transform_delta_y_for_matching = matching_point1_y;

                                    for(int i = 0;i<mpc_pcl->panelpoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_panel_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_panel_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_panel_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_panel_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        mpc_pcl->panelpoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->panelpoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->panelpoint_cloud->points[i].z = 0;
                                        mpc_pcl->panelpoint_cloud->points[i].r = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].g = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].b = 255;
                                    }

                                    // Find optimized waypoints----------
                                    double panel_front_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_front_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_back_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (0.75 + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_back_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (0.75 + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_yaw_direction_x = panel_back_center_x - panel_front_center_x;
                                    double panel_yaw_direction_y = panel_back_center_y - panel_front_center_y;

                                    double prior_way_points[3][2];

                                    if( ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) > 0) &&  ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) < PI)) // right is the fastest
                                    {
                                        optimization_direction_left = false;
                                        copy(&prior_way_points_right[0][0],&prior_way_points_right[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    else
                                    {
                                        optimization_direction_left = true;
                                        copy(&prior_way_points_left[0][0],&prior_way_points_left[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    // ----------------------------------

                                    vector<double> panel_center_to_ugv;

                                    panel_center_to_ugv.push_back(-panel_front_center_x);
                                    panel_center_to_ugv.push_back(-panel_front_center_y);

                                    double angle_from_panel_center;
                                    if(atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0)) < 0)
                                    {
                                        angle_from_panel_center = 2*PI+atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }
                                    else
                                    {
                                        angle_from_panel_center = atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }

                                    for(int i = 0;i<mpc_pcl->waypoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_way_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_way_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_way_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_way_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        double panel_center_to_waypoint_x,panel_center_to_waypoint_y;

                                        panel_center_to_waypoint_x = transform_result_x - panel_front_center_x;
                                        panel_center_to_waypoint_y = transform_result_y - panel_front_center_y;


                                        double outer_product_for_wayupdate = panel_center_to_ugv.at(0)*panel_center_to_waypoint_y - panel_center_to_ugv.at(1)*panel_center_to_waypoint_x;
                                        if(optimization_direction_left)
                                        {
                                            if(outer_product_for_wayupdate < 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }
                                        else
                                        {
                                            if(outer_product_for_wayupdate > 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }


                                        mpc_pcl->waypoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->waypoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->waypoint_cloud->points[i].z = 0;
                                        mpc_pcl->waypoint_cloud->points[i].r = 0;
                                        mpc_pcl->waypoint_cloud->points[i].g = 255;
                                        mpc_pcl->waypoint_cloud->points[i].b = 255;
                                    }
                                    if(current_waypoint_index == 3)
                                    {
                                        current_waypoint_index = 0;
                                    }

                                    if((current_waypoint_index != 2) && (sqrt(mpc_pcl->waypoint_cloud->points[current_waypoint_index].x*mpc_pcl->waypoint_cloud->points[current_waypoint_index].x + mpc_pcl->waypoint_cloud->points[current_waypoint_index].y*mpc_pcl->waypoint_cloud->points[current_waypoint_index].y) < waypoint_converged_margin))
                                    {
                                        current_waypoint_index += 1;
                                    }

                                    double angle_between_panel_yaw_vector = acos((panel_center_to_ugv.at(0)*(-panel_yaw_direction_x)+panel_center_to_ugv.at(1)*(-panel_yaw_direction_y))/(sqrt(panel_center_to_ugv.at(0)*panel_center_to_ugv.at(0) + panel_center_to_ugv.at(1)*panel_center_to_ugv.at(1))*sqrt(panel_yaw_direction_x*panel_yaw_direction_x+panel_yaw_direction_y*panel_yaw_direction_y)));

                                    if(angle_between_panel_yaw_vector < 10.0/180.0*PI)
                                    {
                                        current_waypoint_index = 2;
                                    }

                                    pcl::PointXYZRGBA origin_pt;
                                    origin_pt.x = 0;
                                    origin_pt.y = 0;
                                    origin_pt.z = 0;

                                    waypoint_x = mpc_pcl->waypoint_cloud->points[current_waypoint_index].x;
                                    waypoint_y = mpc_pcl->waypoint_cloud->points[current_waypoint_index].y;

                                }
                            }

                        }
                        else // back
                        {
                            if (coeff[3] != 0)
                            {
                                if ( (coeff[4]/coeff[3]) < 0 )
                                {
                                    matching_point1_index = 3;
                                    matching_point1_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    matching_point2_index = 4;
                                    matching_point2_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    double matching_point_dir_x = matching_point2_x - matching_point1_x;
                                    double matching_point_dir_y = matching_point2_y - matching_point1_y;

                                    double matching_point_dir_norm_x = matching_point_dir_x/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);
                                    double matching_point_dir_norm_y = matching_point_dir_y/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);

                                    double matching_point_dir_angle = atan2(matching_point_dir_norm_y,matching_point_dir_norm_x);

                                    double prior_point_dir_x = prior_panel_points[matching_point2_index][0] - prior_panel_points[matching_point1_index][0];
                                    double prior_point_dir_y = prior_panel_points[matching_point2_index][1] - prior_panel_points[matching_point1_index][1];

                                    double prior_point_dir_norm_x = prior_point_dir_x/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);
                                    double prior_point_dir_norm_y = prior_point_dir_y/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);

                                    double prior_point_dir_angle = atan2(prior_point_dir_norm_y,prior_point_dir_norm_x);

                                    double transform_delta_angle = matching_point_dir_angle - prior_point_dir_angle;

                                    double transform_delta_x_for_prior = -prior_panel_points[matching_point1_index][0];
                                    double transform_delta_y_for_prior = -prior_panel_points[matching_point1_index][1];

                                    double transform_delta_x_for_matching = matching_point1_x;
                                    double transform_delta_y_for_matching = matching_point1_y;

                                    for(int i = 0;i<mpc_pcl->panelpoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_panel_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_panel_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_panel_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_panel_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        mpc_pcl->panelpoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->panelpoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->panelpoint_cloud->points[i].z = 0;
                                        mpc_pcl->panelpoint_cloud->points[i].r = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].g = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].b = 255;
                                    }

                                    // Find optimized waypoints----------
                                    double panel_front_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_front_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_back_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (0.75 + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_back_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (0.75 + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_yaw_direction_x = panel_back_center_x - panel_front_center_x;
                                    double panel_yaw_direction_y = panel_back_center_y - panel_front_center_y;

                                    double prior_way_points[3][2];

                                    if( ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) > 0) &&  ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) < PI)) // right is the fastest
                                    {
                                        optimization_direction_left = false;
                                        copy(&prior_way_points_right[0][0],&prior_way_points_right[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    else
                                    {
                                        optimization_direction_left = true;
                                        copy(&prior_way_points_left[0][0],&prior_way_points_left[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    // ----------------------------------

                                    vector<double> panel_center_to_ugv;

                                    panel_center_to_ugv.push_back(-panel_front_center_x);
                                    panel_center_to_ugv.push_back(-panel_front_center_y);

                                    double angle_from_panel_center;
                                    if(atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0)) < 0)
                                    {
                                        angle_from_panel_center = 2*PI+atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }
                                    else
                                    {
                                        angle_from_panel_center = atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }

                                    for(int i = 0;i<mpc_pcl->waypoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_way_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_way_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_way_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_way_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        double panel_center_to_waypoint_x,panel_center_to_waypoint_y;

                                        panel_center_to_waypoint_x = transform_result_x - panel_front_center_x;
                                        panel_center_to_waypoint_y = transform_result_y - panel_front_center_y;


                                        double outer_product_for_wayupdate = panel_center_to_ugv.at(0)*panel_center_to_waypoint_y - panel_center_to_ugv.at(1)*panel_center_to_waypoint_x;
                                        if(optimization_direction_left)
                                        {
                                            if(outer_product_for_wayupdate < 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }
                                        else
                                        {
                                            if(outer_product_for_wayupdate > 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }


                                        mpc_pcl->waypoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->waypoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->waypoint_cloud->points[i].z = 0;
                                        mpc_pcl->waypoint_cloud->points[i].r = 0;
                                        mpc_pcl->waypoint_cloud->points[i].g = 255;
                                        mpc_pcl->waypoint_cloud->points[i].b = 255;
                                    }
                                    if(current_waypoint_index == 3)
                                    {
                                        current_waypoint_index = 0;
                                    }

                                    if((current_waypoint_index != 2) && (sqrt(mpc_pcl->waypoint_cloud->points[current_waypoint_index].x*mpc_pcl->waypoint_cloud->points[current_waypoint_index].x + mpc_pcl->waypoint_cloud->points[current_waypoint_index].y*mpc_pcl->waypoint_cloud->points[current_waypoint_index].y) < waypoint_converged_margin))
                                    {
                                        current_waypoint_index += 1;
                                    }

                                    double angle_between_panel_yaw_vector = acos((panel_center_to_ugv.at(0)*(-panel_yaw_direction_x)+panel_center_to_ugv.at(1)*(-panel_yaw_direction_y))/(sqrt(panel_center_to_ugv.at(0)*panel_center_to_ugv.at(0) + panel_center_to_ugv.at(1)*panel_center_to_ugv.at(1))*sqrt(panel_yaw_direction_x*panel_yaw_direction_x+panel_yaw_direction_y*panel_yaw_direction_y)));

                                    if(angle_between_panel_yaw_vector < 10.0/180.0*PI)
                                    {
                                        current_waypoint_index = 2;
                                    }

                                    pcl::PointXYZRGBA origin_pt;
                                    origin_pt.x = 0;
                                    origin_pt.y = 0;
                                    origin_pt.z = 0;

                                    waypoint_x = mpc_pcl->waypoint_cloud->points[current_waypoint_index].x;
                                    waypoint_y = mpc_pcl->waypoint_cloud->points[current_waypoint_index].y;
                                }
                                else
                                {
                                    matching_point1_index = 3;
                                    matching_point1_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    matching_point2_index = 4;
                                    matching_point2_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    double matching_point_dir_x = matching_point2_x - matching_point1_x;
                                    double matching_point_dir_y = matching_point2_y - matching_point1_y;

                                    double matching_point_dir_norm_x = matching_point_dir_x/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);
                                    double matching_point_dir_norm_y = matching_point_dir_y/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);

                                    double matching_point_dir_angle = atan2(matching_point_dir_norm_y,matching_point_dir_norm_x);

                                    double prior_point_dir_x = prior_panel_points[matching_point2_index][0] - prior_panel_points[matching_point1_index][0];
                                    double prior_point_dir_y = prior_panel_points[matching_point2_index][1] - prior_panel_points[matching_point1_index][1];

                                    double prior_point_dir_norm_x = prior_point_dir_x/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);
                                    double prior_point_dir_norm_y = prior_point_dir_y/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);

                                    double prior_point_dir_angle = atan2(prior_point_dir_norm_y,prior_point_dir_norm_x);

                                    double transform_delta_angle = matching_point_dir_angle - prior_point_dir_angle;

                                    double transform_delta_x_for_prior = -prior_panel_points[matching_point1_index][0];
                                    double transform_delta_y_for_prior = -prior_panel_points[matching_point1_index][1];

                                    double transform_delta_x_for_matching = matching_point1_x;
                                    double transform_delta_y_for_matching = matching_point1_y;

                                    for(int i = 0;i<mpc_pcl->panelpoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_panel_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_panel_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_panel_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_panel_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        mpc_pcl->panelpoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->panelpoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->panelpoint_cloud->points[i].z = 0;
                                        mpc_pcl->panelpoint_cloud->points[i].r = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].g = 255;
                                        mpc_pcl->panelpoint_cloud->points[i].b = 255;
                                    }

                                    // Find optimized waypoints----------
                                    double panel_front_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_front_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_back_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (0.75 + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double panel_back_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (0.75 + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_yaw_direction_x = panel_back_center_x - panel_front_center_x;
                                    double panel_yaw_direction_y = panel_back_center_y - panel_front_center_y;

                                    double prior_way_points[3][2];

                                    if( ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) > 0) &&  ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) < PI)) // right is the fastest
                                    {
                                        optimization_direction_left = false;
                                        copy(&prior_way_points_right[0][0],&prior_way_points_right[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    else
                                    {
                                        optimization_direction_left = true;
                                        copy(&prior_way_points_left[0][0],&prior_way_points_left[0][0] + 6,&prior_way_points[0][0]);
                                    }
                                    // ----------------------------------

                                    vector<double> panel_center_to_ugv;

                                    panel_center_to_ugv.push_back(-panel_front_center_x);
                                    panel_center_to_ugv.push_back(-panel_front_center_y);

                                    double angle_from_panel_center;
                                    if(atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0)) < 0)
                                    {
                                        angle_from_panel_center = 2*PI+atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }
                                    else
                                    {
                                        angle_from_panel_center = atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                    }

                                    for(int i = 0;i<mpc_pcl->waypoint_cloud->points.size();i++)
                                    {
                                        double transform_result_x = (prior_way_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_way_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                        double transform_result_y = (prior_way_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_way_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                        double panel_center_to_waypoint_x,panel_center_to_waypoint_y;

                                        panel_center_to_waypoint_x = transform_result_x - panel_front_center_x;
                                        panel_center_to_waypoint_y = transform_result_y - panel_front_center_y;


                                        double outer_product_for_wayupdate = panel_center_to_ugv.at(0)*panel_center_to_waypoint_y - panel_center_to_ugv.at(1)*panel_center_to_waypoint_x;
                                        if(optimization_direction_left)
                                        {
                                            if(outer_product_for_wayupdate < 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }
                                        else
                                        {
                                            if(outer_product_for_wayupdate > 0)
                                            {
                                                current_waypoint_index = i+1;
                                            }
                                        }


                                        mpc_pcl->waypoint_cloud->points[i].x = transform_result_x;
                                        mpc_pcl->waypoint_cloud->points[i].y = transform_result_y;
                                        mpc_pcl->waypoint_cloud->points[i].z = 0;
                                        mpc_pcl->waypoint_cloud->points[i].r = 0;
                                        mpc_pcl->waypoint_cloud->points[i].g = 255;
                                        mpc_pcl->waypoint_cloud->points[i].b = 255;
                                    }
                                    if(current_waypoint_index == 3)
                                    {
                                        current_waypoint_index = 0;
                                    }

                                    if((current_waypoint_index != 2) && (sqrt(mpc_pcl->waypoint_cloud->points[current_waypoint_index].x*mpc_pcl->waypoint_cloud->points[current_waypoint_index].x + mpc_pcl->waypoint_cloud->points[current_waypoint_index].y*mpc_pcl->waypoint_cloud->points[current_waypoint_index].y) < waypoint_converged_margin))
                                    {
                                        current_waypoint_index += 1;
                                    }

                                    double angle_between_panel_yaw_vector = acos((panel_center_to_ugv.at(0)*(-panel_yaw_direction_x)+panel_center_to_ugv.at(1)*(-panel_yaw_direction_y))/(sqrt(panel_center_to_ugv.at(0)*panel_center_to_ugv.at(0) + panel_center_to_ugv.at(1)*panel_center_to_ugv.at(1))*sqrt(panel_yaw_direction_x*panel_yaw_direction_x+panel_yaw_direction_y*panel_yaw_direction_y)));

                                    if(angle_between_panel_yaw_vector < 10.0/180.0*PI)
                                    {
                                        current_waypoint_index = 2;
                                    }

                                    pcl::PointXYZRGBA origin_pt;
                                    origin_pt.x = 0;
                                    origin_pt.y = 0;
                                    origin_pt.z = 0;

                                    waypoint_x = mpc_pcl->waypoint_cloud->points[current_waypoint_index].x;
                                    waypoint_y = mpc_pcl->waypoint_cloud->points[current_waypoint_index].y;
                                }
                            }



                        }
                    }
                    else
                    {

                    }

                }
                else if(((2.0*maximum_dist_from_ransac_mean) > 0.35) && ((2.0*maximum_dist_from_ransac_mean) < 0.65))// side
                {
                    find_panel_point = true;
                    if( (coeff[3]*coeff1[3] + coeff[4]*coeff1[4]) > 0.5) // 0ch and 10.67 ch is parrell
                    {
                        double waypoint_direct_vec_x = ransac_line1_mean_x - ransac_line_mean_x;
                        double waypoint_direct_vec_y = ransac_line1_mean_y - ransac_line_mean_y;

                        double waypoint_direct_vec_x_norm = waypoint_direct_vec_x/sqrt(waypoint_direct_vec_x*waypoint_direct_vec_x + waypoint_direct_vec_y*waypoint_direct_vec_y);
                        double waypoint_direct_vec_y_norm = waypoint_direct_vec_y/sqrt(waypoint_direct_vec_x*waypoint_direct_vec_x + waypoint_direct_vec_y*waypoint_direct_vec_y);

                        double outer_product_result = ransac_line_mean_x*waypoint_direct_vec_y_norm - ransac_line_mean_y*waypoint_direct_vec_x_norm;

                        if (coeff[3] != 0)
                        {
                            if ( (coeff[4]/coeff[3]) < 0 )
                            {
                                if(outer_product_result < 0) // Left side
                                {
                                    matching_point1_index = 4;
                                    matching_point1_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    matching_point2_index = 5;
                                    matching_point2_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);
                                }
                                else // right side
                                {
                                    matching_point1_index = 2;
                                    matching_point1_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    matching_point2_index = 3;
                                    matching_point2_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);
                                }

                                double matching_point_dir_x = matching_point2_x - matching_point1_x;
                                double matching_point_dir_y = matching_point2_y - matching_point1_y;

                                double matching_point_dir_norm_x = matching_point_dir_x/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);
                                double matching_point_dir_norm_y = matching_point_dir_y/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);

                                double matching_point_dir_angle = atan2(matching_point_dir_norm_y,matching_point_dir_norm_x);

                                double prior_point_dir_x = prior_panel_points[matching_point2_index][0] - prior_panel_points[matching_point1_index][0];
                                double prior_point_dir_y = prior_panel_points[matching_point2_index][1] - prior_panel_points[matching_point1_index][1];

                                double prior_point_dir_norm_x = prior_point_dir_x/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);
                                double prior_point_dir_norm_y = prior_point_dir_y/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);

                                double prior_point_dir_angle = atan2(prior_point_dir_norm_y,prior_point_dir_norm_x);

                                double transform_delta_angle = matching_point_dir_angle - prior_point_dir_angle;

                                double transform_delta_x_for_prior = -prior_panel_points[matching_point1_index][0];
                                double transform_delta_y_for_prior = -prior_panel_points[matching_point1_index][1];

                                double transform_delta_x_for_matching = matching_point1_x;
                                double transform_delta_y_for_matching = matching_point1_y;

                                for(int i = 0;i<mpc_pcl->panelpoint_cloud->points.size();i++)
                                {
                                    double transform_result_x = (prior_panel_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_panel_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double transform_result_y = (prior_panel_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_panel_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    mpc_pcl->panelpoint_cloud->points[i].x = transform_result_x;
                                    mpc_pcl->panelpoint_cloud->points[i].y = transform_result_y;
                                    mpc_pcl->panelpoint_cloud->points[i].z = 0;
                                    mpc_pcl->panelpoint_cloud->points[i].r = 255;
                                    mpc_pcl->panelpoint_cloud->points[i].g = 255;
                                    mpc_pcl->panelpoint_cloud->points[i].b = 255;
                                }

                                // Find optimized waypoints----------
                                double panel_front_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                double panel_front_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                double panel_back_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (0.75 + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                double panel_back_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (0.75 + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                double panel_yaw_direction_x = panel_back_center_x - panel_front_center_x;
                                double panel_yaw_direction_y = panel_back_center_y - panel_front_center_y;

                                double prior_way_points[3][2];

                                if( ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) > 0) &&  ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) < PI)) // right is the fastest
                                {
                                    optimization_direction_left = false;
                                    copy(&prior_way_points_right[0][0],&prior_way_points_right[0][0] + 6,&prior_way_points[0][0]);
                                }
                                else
                                {
                                    optimization_direction_left = true;
                                    copy(&prior_way_points_left[0][0],&prior_way_points_left[0][0] + 6,&prior_way_points[0][0]);
                                }
                                // ----------------------------------

                                vector<double> panel_center_to_ugv;

                                panel_center_to_ugv.push_back(-panel_front_center_x);
                                panel_center_to_ugv.push_back(-panel_front_center_y);

                                double angle_from_panel_center;
                                if(atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0)) < 0)
                                {
                                    angle_from_panel_center = 2*PI+atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                }
                                else
                                {
                                    angle_from_panel_center = atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                }

                                for(int i = 0;i<mpc_pcl->waypoint_cloud->points.size();i++)
                                {
                                    double transform_result_x = (prior_way_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_way_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double transform_result_y = (prior_way_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_way_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_center_to_waypoint_x,panel_center_to_waypoint_y;

                                    panel_center_to_waypoint_x = transform_result_x - panel_front_center_x;
                                    panel_center_to_waypoint_y = transform_result_y - panel_front_center_y;


                                    double outer_product_for_wayupdate = panel_center_to_ugv.at(0)*panel_center_to_waypoint_y - panel_center_to_ugv.at(1)*panel_center_to_waypoint_x;
                                    if(optimization_direction_left)
                                    {
                                        if(outer_product_for_wayupdate < 0)
                                        {
                                            current_waypoint_index = i+1;
                                        }
                                    }
                                    else
                                    {
                                        if(outer_product_for_wayupdate > 0)
                                        {
                                            current_waypoint_index = i+1;
                                        }
                                    }


                                    mpc_pcl->waypoint_cloud->points[i].x = transform_result_x;
                                    mpc_pcl->waypoint_cloud->points[i].y = transform_result_y;
                                    mpc_pcl->waypoint_cloud->points[i].z = 0;
                                    mpc_pcl->waypoint_cloud->points[i].r = 0;
                                    mpc_pcl->waypoint_cloud->points[i].g = 255;
                                    mpc_pcl->waypoint_cloud->points[i].b = 255;
                                }
                                if(current_waypoint_index == 3)
                                {
                                    current_waypoint_index = 0;
                                }

                                if((current_waypoint_index != 2) && (sqrt(mpc_pcl->waypoint_cloud->points[current_waypoint_index].x*mpc_pcl->waypoint_cloud->points[current_waypoint_index].x + mpc_pcl->waypoint_cloud->points[current_waypoint_index].y*mpc_pcl->waypoint_cloud->points[current_waypoint_index].y) < waypoint_converged_margin))
                                {
                                    current_waypoint_index += 1;
                                }

                                double angle_between_panel_yaw_vector = acos((panel_center_to_ugv.at(0)*(-panel_yaw_direction_x)+panel_center_to_ugv.at(1)*(-panel_yaw_direction_y))/(sqrt(panel_center_to_ugv.at(0)*panel_center_to_ugv.at(0) + panel_center_to_ugv.at(1)*panel_center_to_ugv.at(1))*sqrt(panel_yaw_direction_x*panel_yaw_direction_x+panel_yaw_direction_y*panel_yaw_direction_y)));

                                if(angle_between_panel_yaw_vector < 10.0/180.0*PI)
                                {
                                    current_waypoint_index = 2;
                                }

                                pcl::PointXYZRGBA origin_pt;
                                origin_pt.x = 0;
                                origin_pt.y = 0;
                                origin_pt.z = 0;

                                waypoint_x = mpc_pcl->waypoint_cloud->points[current_waypoint_index].x;
                                waypoint_y = mpc_pcl->waypoint_cloud->points[current_waypoint_index].y;
                            }
                            else
                            {
                                if(outer_product_result < 0) // Left side
                                {
                                    matching_point1_index = 4;
                                    matching_point1_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    matching_point2_index = 5;
                                    matching_point2_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);
                                }
                                else // right side
                                {
                                    matching_point1_index = 2;
                                    matching_point1_x = ransac_line_mean_x - maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point1_y = ransac_line_mean_y - maximum_dist_from_ransac_mean*abs(coeff[4]);

                                    matching_point2_index = 3;
                                    matching_point2_x = ransac_line_mean_x + maximum_dist_from_ransac_mean*abs(coeff[3]);
                                    matching_point2_y = ransac_line_mean_y + maximum_dist_from_ransac_mean*abs(coeff[4]);
                                }

                                double matching_point_dir_x = matching_point2_x - matching_point1_x;
                                double matching_point_dir_y = matching_point2_y - matching_point1_y;

                                double matching_point_dir_norm_x = matching_point_dir_x/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);
                                double matching_point_dir_norm_y = matching_point_dir_y/sqrt(matching_point_dir_x*matching_point_dir_x + matching_point_dir_y*matching_point_dir_y);

                                double matching_point_dir_angle = atan2(matching_point_dir_norm_y,matching_point_dir_norm_x);

                                double prior_point_dir_x = prior_panel_points[matching_point2_index][0] - prior_panel_points[matching_point1_index][0];
                                double prior_point_dir_y = prior_panel_points[matching_point2_index][1] - prior_panel_points[matching_point1_index][1];

                                double prior_point_dir_norm_x = prior_point_dir_x/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);
                                double prior_point_dir_norm_y = prior_point_dir_y/sqrt(prior_point_dir_x*prior_point_dir_x + prior_point_dir_y*prior_point_dir_y);

                                double prior_point_dir_angle = atan2(prior_point_dir_norm_y,prior_point_dir_norm_x);

                                double transform_delta_angle = matching_point_dir_angle - prior_point_dir_angle;

                                double transform_delta_x_for_prior = -prior_panel_points[matching_point1_index][0];
                                double transform_delta_y_for_prior = -prior_panel_points[matching_point1_index][1];

                                double transform_delta_x_for_matching = matching_point1_x;
                                double transform_delta_y_for_matching = matching_point1_y;

                                for(int i = 0;i<mpc_pcl->panelpoint_cloud->points.size();i++)
                                {
                                    double transform_result_x = (prior_panel_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_panel_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double transform_result_y = (prior_panel_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_panel_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    mpc_pcl->panelpoint_cloud->points[i].x = transform_result_x;
                                    mpc_pcl->panelpoint_cloud->points[i].y = transform_result_y;
                                    mpc_pcl->panelpoint_cloud->points[i].z = 0;
                                    mpc_pcl->panelpoint_cloud->points[i].r = 255;
                                    mpc_pcl->panelpoint_cloud->points[i].g = 255;
                                    mpc_pcl->panelpoint_cloud->points[i].b = 255;
                                }

                                // Find optimized waypoints----------
                                double panel_front_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                double panel_front_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                double panel_back_center_x = (0.5 + transform_delta_x_for_prior)*cos(transform_delta_angle) - (0.75 + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                double panel_back_center_y = (0.5 + transform_delta_x_for_prior)*sin(transform_delta_angle) + (0.75 + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                double panel_yaw_direction_x = panel_back_center_x - panel_front_center_x;
                                double panel_yaw_direction_y = panel_back_center_y - panel_front_center_y;

                                double prior_way_points[3][2];

                                if( ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) > 0) &&  ((atan2(panel_yaw_direction_y,panel_yaw_direction_x)) < PI)) // right is the fastest
                                {
                                    optimization_direction_left = false;
                                    copy(&prior_way_points_right[0][0],&prior_way_points_right[0][0] + 6,&prior_way_points[0][0]);
                                }
                                else
                                {
                                    optimization_direction_left = true;
                                    copy(&prior_way_points_left[0][0],&prior_way_points_left[0][0] + 6,&prior_way_points[0][0]);
                                }
                                // ----------------------------------

                                vector<double> panel_center_to_ugv;

                                panel_center_to_ugv.push_back(-panel_front_center_x);
                                panel_center_to_ugv.push_back(-panel_front_center_y);

                                double angle_from_panel_center;
                                if(atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0)) < 0)
                                {
                                    angle_from_panel_center = 2*PI+atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                }
                                else
                                {
                                    angle_from_panel_center = atan2(panel_center_to_ugv.at(1),panel_center_to_ugv.at(0));
                                }

                                for(int i = 0;i<mpc_pcl->waypoint_cloud->points.size();i++)
                                {
                                    double transform_result_x = (prior_way_points[i][0] + transform_delta_x_for_prior)*cos(transform_delta_angle) - (prior_way_points[i][1] + transform_delta_y_for_prior)*sin(transform_delta_angle) + transform_delta_x_for_matching;
                                    double transform_result_y = (prior_way_points[i][0] + transform_delta_x_for_prior)*sin(transform_delta_angle) + (prior_way_points[i][1] + transform_delta_y_for_prior)*cos(transform_delta_angle) + transform_delta_y_for_matching;

                                    double panel_center_to_waypoint_x,panel_center_to_waypoint_y;

                                    panel_center_to_waypoint_x = transform_result_x - panel_front_center_x;
                                    panel_center_to_waypoint_y = transform_result_y - panel_front_center_y;


                                    double outer_product_for_wayupdate = panel_center_to_ugv.at(0)*panel_center_to_waypoint_y - panel_center_to_ugv.at(1)*panel_center_to_waypoint_x;
                                    if(optimization_direction_left)
                                    {
                                        if(outer_product_for_wayupdate < 0)
                                        {
                                            current_waypoint_index = i+1;
                                        }
                                    }
                                    else
                                    {
                                        if(outer_product_for_wayupdate > 0)
                                        {
                                            current_waypoint_index = i+1;
                                        }
                                    }


                                    mpc_pcl->waypoint_cloud->points[i].x = transform_result_x;
                                    mpc_pcl->waypoint_cloud->points[i].y = transform_result_y;
                                    mpc_pcl->waypoint_cloud->points[i].z = 0;
                                    mpc_pcl->waypoint_cloud->points[i].r = 0;
                                    mpc_pcl->waypoint_cloud->points[i].g = 255;
                                    mpc_pcl->waypoint_cloud->points[i].b = 255;
                                }

                                if(current_waypoint_index == 3)
                                {
                                    current_waypoint_index = 0;
                                }

                                if((current_waypoint_index != 2) && (sqrt(mpc_pcl->waypoint_cloud->points[current_waypoint_index].x*mpc_pcl->waypoint_cloud->points[current_waypoint_index].x + mpc_pcl->waypoint_cloud->points[current_waypoint_index].y*mpc_pcl->waypoint_cloud->points[current_waypoint_index].y) < waypoint_converged_margin))
                                {
                                    current_waypoint_index += 1;
                                }

                                double angle_between_panel_yaw_vector = acos((panel_center_to_ugv.at(0)*(-panel_yaw_direction_x)+panel_center_to_ugv.at(1)*(-panel_yaw_direction_y))/(sqrt(panel_center_to_ugv.at(0)*panel_center_to_ugv.at(0) + panel_center_to_ugv.at(1)*panel_center_to_ugv.at(1))*sqrt(panel_yaw_direction_x*panel_yaw_direction_x+panel_yaw_direction_y*panel_yaw_direction_y)));

                                if(angle_between_panel_yaw_vector < 10.0/180.0*PI)
                                {
                                    current_waypoint_index = 2;
                                }

                                pcl::PointXYZRGBA origin_pt;
                                origin_pt.x = 0;
                                origin_pt.y = 0;
                                origin_pt.z = 0;

                                waypoint_x = mpc_pcl->waypoint_cloud->points[current_waypoint_index].x;
                                waypoint_y = mpc_pcl->waypoint_cloud->points[current_waypoint_index].y;
                            }
                        }

                    }
                    else
                    {

                    }
                }
                else // no panel detect
                {
                    find_panel_point = false;
                }

                (*final) += (*final1);
                mpc_pcl->viewer->updatePointCloud(final,"cloud");
                mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");
                mpc_pcl->viewer->updatePointCloud(mpc_pcl->waypoint_cloud ,"waypoint_cloud");
                mpc_pcl->viewer->updatePointCloud(mpc_pcl->panelpoint_cloud,"panelpoint_cloud");

            }


            fl_parser_complete = true;

            emit SignalVelodyneParser(fl_parser_complete);

            memset(buffer, 0, sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

            count = 0;

            mtx_pcl_class.unlock();


        }

        prev_deg = deg;

        if (count >= VELODYNE_TOTAL_PACKET_NUMBER){
            count = 0;
        }

        count++;

    }

    return true;
}

vector<double> CVelodyne::GetWaypoint()
{
    vector<double> way_point_vec;
    way_point_vec.push_back(waypoint_x);
    way_point_vec.push_back(waypoint_y);

    return way_point_vec;
}

bool CVelodyne::GetPanelFindStatus()
{
    return find_panel_point;
}

bool CVelodyne::GetUGVTurnDirection()
{
    return optimization_direction_left;
}

std::vector<double> CVelodyne::GetPanelCenterLoc()
{

    std::vector<double> mean_panel_loc;

    mean_panel_loc.push_back(mean_panel_x);
    mean_panel_loc.push_back(mean_panel_y);
    mean_panel_loc.push_back(mean_dist);

    return mean_panel_loc;
}

std::vector<double> CVelodyne::GetLRFPanelInfo()
{
    mtx_pcl_class.lock();
    std::vector<double> panel_info;

    panel_info.push_back(lrf_slope_norm_x);
    panel_info.push_back(lrf_slope_norm_y);
    panel_info.push_back(lrf_panel_length);
    panel_info.push_back(lrf_ransac_line_mean_x);
    panel_info.push_back(lrf_ransac_line_mean_y);
    mtx_pcl_class.unlock();
    return panel_info;
}

bool CVelodyne::SetLRFDataToPCL(long *_lrf_data,int _num_of_points)
{
    mtx_pcl_class.lock();
    mpc_pcl->lrf_cloud->clear();
    for(int i = 0;i<_num_of_points;i++)
    {
        if(_lrf_data[i] <= 2000)
        {
            pcl::PointXYZRGBA point_lrf;
            point_lrf.x = -(_lrf_data[i]*cos(0.25/180.0*PI*i)*0.001-0.35);
            point_lrf.y = -(_lrf_data[i]*sin(0.25/180.0*PI*i)*0.001+0.25);
            point_lrf.z = 0;
            point_lrf.r = 0;
            point_lrf.g = 255;
            point_lrf.b = 0;
            mpc_pcl->lrf_cloud->points.push_back(point_lrf);
        }
    }
    mtx_pcl_class.unlock();
    return true;
}

void CVelodyne::SetVelodyneRange(double _range)
{
    mtx_pcl_class.lock();
    velodyne_range = _range;
    mtx_pcl_class.unlock();
}

void CVelodyne::SetVelodyneMode(VELODYNE_MODE _mode)
{
    mtx_pcl_class.lock();
    velodyne_mode = _mode;
    if(velodyne_mode == VELODYNE_MODE_DRIVING)
    {
        velodyne_range = velodyne_range_driving;
    }
    else
    {
        velodyne_range = velodyne_range_parking;
    }
    mtx_pcl_class.unlock();
}

bool CVelodyne::GetLRFPanelFindStatus()
{
    return lrf_find_panel;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CVelodyne::run(){

    RunVelodyne();
}








