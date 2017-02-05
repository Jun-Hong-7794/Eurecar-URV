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

    double clustering_tolerence = 0.1;
    double clustering_count_tolerence = 150;
    double parking_distance = 1.0;


    while(fl_velodyne_thread){


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

            {
                memcpy(mpc_pcl->m_velodyne_data_ary, buffer,
                    sizeof(VELODYNE_DATA)*VELODYNE_TOTAL_PACKET_NUMBER);

                mpc_pcl->Set_Velodyne_Data(mpc_pcl->m_x_data,mpc_pcl->m_y_data,mpc_pcl->m_z_data);

                mpc_pcl->cloud->points.resize(VELODYNE_LASERS_NUM*VELODYNE_TOTAL_PACKET_NUMBER*VELODYNE_BOLCKS_NUM);

                point_index = 0;
                panel_point_index = 0;


                double minimum_z = 0;
                double tolerence = 0.3;



                pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter(true);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ei_result (new pcl::PointCloud<pcl::PointXYZRGBA>);



                //for 0 index point cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr parral_points (new pcl::PointCloud<pcl::PointXYZ>);
                //for projected points to image
                cv::Mat projected_img = cv::Mat::zeros(200,400,CV_8UC1);
                cv::Mat projectedline_img = cv::Mat::zeros(200,400,CV_8UC3);

                // Jung's projected img
                cv::Mat xy_plane_image = cv::Mat::zeros(600,600,CV_8UC1);

                parral_points->clear();



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


//                            else if((firing_vertical_angle[k] > -10 && firing_vertical_angle[k] < -8 || firing_vertical_angle[k] > 8 && firing_vertical_angle[k] < 10 )
//                                    && (mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] > 700)
//                                    && (mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] < 2000)
//                                    && (mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)] < 0 )){

//                                sum_panel_x += mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
//                                sum_panel_y += mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
//                                sum_dist += mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

//                                panel_point_index++;

//                                mpc_pcl->cloud->points[point_index].r = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
//                                mpc_pcl->cloud->points[point_index].g = 255;// *(1024 * rand () / (RAND_MAX + 1.0f));
//                                mpc_pcl->cloud->points[point_index].b = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
//                            }

                            else{
                                mpc_pcl->cloud->points[point_index].r = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].g = 0;// *(1024 * rand () / (RAND_MAX + 1.0f));
                                mpc_pcl->cloud->points[point_index].b = 255;// *(1024 * rand () / (RAND_MAX + 1.0f));
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


                int clustering_index = 0;
                int clustering_member_count = 0;


                for(int k = 0;k < VELODYNE_LASERS_NUM;k++){
                    for(int i=0; i< VELODYNE_TOTAL_PACKET_NUMBER;i++){
                        for(int j=0; j < VELODYNE_BOLCKS_NUM;j++){
                            mpc_pcl->cloud->points[point_index].x = mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                            mpc_pcl->cloud->points[point_index].y = mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                            mpc_pcl->cloud->points[point_index].z = mpc_pcl->m_z_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

                            if((std::abs((mpc_pcl->cloud->points[point_index].z - minimum_z)) > tolerence) && ((mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] < 3000)))
                            {
                                if((firing_vertical_angle[k] == 0))
                                {

                                    adjacent_dist = std::sqrt(std::pow(mpc_pcl->cloud->points[point_index].x - past_point_x,2) + std::pow(mpc_pcl->cloud->points[point_index].y - past_point_y,2) + std::pow(mpc_pcl->cloud->points[point_index].z - past_point_z,2)) ;

                                    past_point_x = mpc_pcl->cloud->points[point_index].x;
                                    past_point_y = mpc_pcl->cloud->points[point_index].y;
                                    past_point_z = mpc_pcl->cloud->points[point_index].z;

                                    if(adjacent_dist >0)
                                    {

                                        sum_panel_x += mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                                        sum_panel_y += mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                                        sum_dist += mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

                                        panel_point_index++;


                                        if(adjacent_dist < clustering_tolerence)
                                        {
                                            (*inliers).indices.push_back(point_index);
                                            clustering_member_count++;
                                            mpc_pcl->cloud->points[point_index].r = cv::saturate_cast<uchar>(255 - clustering_index*10);
                                            mpc_pcl->cloud->points[point_index].g = cv::saturate_cast<uchar>(128 + clustering_index*10);
                                            mpc_pcl->cloud->points[point_index].b = cv::saturate_cast<uchar>(clustering_index*10);
                                        }
                                        else
                                        {
                                            if(clustering_member_count < clustering_count_tolerence)
                                            {
                                                (*inliers).indices.erase((*inliers).indices.end()-clustering_member_count, (*inliers).indices.end());
                                            }
                                            else
                                            {

                                            }
                                            clustering_member_count = 0;
                                            clustering_index++;
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

//                //////////////////////////////////////////////projection//////////////////////////////////////////////projection
//                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

//                //create a set of planer coefficient x,y,z
//                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
//                coefficients->values.resize(4);
//                coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0;
//                coefficients->values[2] = 1.0;

//                pcl::ProjectInliers<pcl::PointXYZ> proj;
//                proj.setModelType(pcl::SACMODEL_PLANE);
//                proj.setInputCloud(parral_points);
//                proj.setModelCoefficients(coefficients);
//                proj.filter(*cloud_projected);

//                for(int i=0;i <cloud_projected->points.size(); i++)
//                {
//                    projected_img.at<uchar>(cv::Point((cloud_projected->points[i].y * 100),(cloud_projected->points[i].x + 2) *100)) = 255;
//                }

////                cv::Canny(projected_img,projected_img,50,200,3);

//                cv::cvtColor(projected_img,projectedline_img,cv::COLOR_GRAY2BGR);


//                vector<cv::Vec4i> lines;

//                // NEED TO BE TUNED THRESHOLD LEVEL FOR PANAL
//                cv::HoughLinesP(projected_img,lines,1,CV_PI/180,20,30,10);

//                double max_len =0;
//                double len =0;
//                double max_len_slope =0;
//                int max_len_index =0;

//                for(int i=0; i<lines.size(); i++)
//                {
//                    cv::Vec4i l = lines[i];
//                    len = cv::norm(cv::Point(l[2],l[3]) - cv::Point(l[0],l[1]));
//                    if(len > max_len)
//                    {
//                        max_len = len;
//                        max_len_index = i;
//                    }
//                }
//                for(int i=0; i<lines.size(); i++)
//                {
//                    cv::Vec4i l = lines[i];
//                    if( i == max_len_index)
//                    {
//                        cv::line(projectedline_img,cv::Point(l[0],l[1]),cv::Point(l[2],l[3]),cv::Scalar(0,0,255),3,2);
//                        max_len_slope = atan((-1)*((cv::Point(l[2],l[3]).y - cv::Point(l[0],l[1]).y) / (double)(cv::Point(l[2],l[3]).x - cv::Point(l[0],l[1]).x))) * 180 /3.14 ;
//                        if(max_len_slope < 0)
//                            max_len_slope += 90;
//                    }
//                    else
//                        cv::line(projectedline_img,cv::Point(l[0],l[1]),cv::Point(l[2],l[3]),cv::Scalar(0,255,0),3,2);
//                }

//                std::cout << max_len_slope << std::endl;

//                cv::imwrite("projected line img.jpg",projectedline_img);
//                //////////////////////////////////////////////////////////////////////////////////////////////////////projection


                eifilter.setInputCloud(mpc_pcl->cloud);
                eifilter.setIndices(inliers);
                eifilter.filter(*ei_result);

                for(int i = 0; i < ei_result->points.size();i++)
                {
                    int x = 300 + 100.0 * ei_result->points[i].y;
                    int y = 300 + 100.0 * ei_result->points[i].x;
                    xy_plane_image.at<uchar>(y,x) = 255;
                }

                cv::imwrite("xy_plane_image.jpg",xy_plane_image);

                vector<cv::Vec4i> lines;

                // NEED TO BE TUNED THRESHOLD LEVEL FOR PANAL
                cv::HoughLinesP(xy_plane_image,lines,1,CV_PI/180,20,30,10);

                double max_len =0;
                double len =0;
                double max_len_slope =0;

                double parking_tr_x;
                double parking_tr_y;
                for(int i=0; i<lines.size(); i++)
                {
                    cv::Vec4i l = lines[i];
                    len = cv::norm(cv::Point(l[2],l[3]) - cv::Point(l[0],l[1]));
                    if(len > max_len)
                    {
                        max_len = len;
                        max_len_slope = atan((-1)*((cv::Point(l[2],l[3]).y - cv::Point(l[0],l[1]).y) / (double)(cv::Point(l[2],l[3]).x - cv::Point(l[0],l[1]).x))) ;

                        parking_tr_x = 0.01*(((double)(cv::Point(l[2],l[3]).y + cv::Point(l[0],l[1]).y)/2.0) - 100.0*parking_distance*std::sin(max_len_slope - PI/2.0) - 300.0);
                        parking_tr_y = 0.01*(((double)(cv::Point(l[2],l[3]).x + cv::Point(l[0],l[1]).x)/2.0) + 100.0*parking_distance*std::cos(max_len_slope - PI/2.0) - 300.0);

                    }
                }

                (*inliers).indices.clear();

                mpc_pcl->viewer->updatePointCloud(ei_result,"cloud");
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








