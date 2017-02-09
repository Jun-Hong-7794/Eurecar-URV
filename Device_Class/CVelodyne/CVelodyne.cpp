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

    double clustering_tolerence = 0.1;
    double clustering_count_tolerence = 50;
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


                double minimum_z = 0.;
                double tolerence = 0.;



                pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter(true);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ei_result (new pcl::PointCloud<pcl::PointXYZRGBA>);

                pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter2(true);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ei_result2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

                // Jung's projected img
                cv::Mat xy_plane_image_0 = cv::Mat::zeros(600,600,CV_8UC1);
                cv::Mat xy_plane_image_10 = cv::Mat::zeros(600,600,CV_8UC1);

                cv::Mat xy_plane_color_image_0 = cv::Mat::zeros(600,600,CV_8UC3);
                cv::Mat xy_plane_color_image_10 = cv::Mat::zeros(600,600,CV_8UC3);


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

                double past_point_x2 = .0;
                double past_point_y2 = .0;
                double past_point_z2 = .0;
                double adjacent_dist2 = .0;


                int clustering_index = 0;
                int clustering_index2 = 0;
                int clustering_member_count = 0;
                int clustering_member_count2 = 0;


                for(int k = 0;k < VELODYNE_LASERS_NUM;k++){
                    for(int i=0; i< VELODYNE_TOTAL_PACKET_NUMBER;i++){
                        for(int j=0; j < VELODYNE_BOLCKS_NUM;j++){
                            mpc_pcl->cloud->points[point_index].x = mpc_pcl->m_x_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                            mpc_pcl->cloud->points[point_index].y = mpc_pcl->m_y_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;
                            mpc_pcl->cloud->points[point_index].z = mpc_pcl->m_z_data[k][(j + i*VELODYNE_BOLCKS_NUM)]*0.001;

                            if((std::abs((mpc_pcl->cloud->points[point_index].z - minimum_z)) > tolerence) && ((mpc_pcl->m_dist_data[k][(j + i*VELODYNE_BOLCKS_NUM)] < 2000)) && mpc_pcl->cloud->points[point_index].x < 0)
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
                                            clustering_member_count++;
//                                            mpc_pcl->cloud->points[point_index].r = cv::saturate_cast<uchar>(255 - clustering_index*10);
//                                            mpc_pcl->cloud->points[point_index].g = cv::saturate_cast<uchar>(128 + clustering_index*10);
//                                            mpc_pcl->cloud->points[point_index].b = cv::saturate_cast<uchar>(clustering_index*10);
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

                                    if(adjacent_dist2 >0)
                                    {
                                        if(adjacent_dist2 < clustering_tolerence)
                                        {
                                            (*inliers2).indices.push_back(point_index);
                                            clustering_member_count2++;
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

                for(int i = 0; i < ei_result->points.size();i++)
                {
                    int x = 300 + 100.0 * ei_result->points[i].y;
                    int y = 300 + 100.0 * ei_result->points[i].x;

                    xy_plane_image_0.at<uchar>(y,x) = 255;

                    xy_plane_image_0.at<uchar>(y+1,x) = 255;
                    xy_plane_image_0.at<uchar>(y-1,x) = 255;
                    xy_plane_image_0.at<uchar>(y,x+1) = 255;
                    xy_plane_image_0.at<uchar>(y,x-1) = 255;

                }

                for(int i = 0; i < ei_result2->points.size();i++)
                {
                    int x2 = 300 + 100.0 * ei_result2->points[i].y;
                    int y2 = 300 + 100.0 * ei_result2->points[i].x;

                    xy_plane_image_10.at<uchar>(y2,x2) = 255;

                    xy_plane_image_10.at<uchar>(y2+1,x2) = 255;
                    xy_plane_image_10.at<uchar>(y2-1,x2) = 255;
                    xy_plane_image_10.at<uchar>(y2,x2+1) = 255;
                    xy_plane_image_10.at<uchar>(y2,x2-1) = 255;
                }

                cv::imwrite("xy_plane_image0.jpg",xy_plane_image_0);
                cv::imwrite("xy_plane_image10.jpg",xy_plane_image_10);

//                cv::erode(xy_plane_color_image_0,xy_plane_color_image_0);
//                cv::erode(xy_plane_color_image_10,xy_plane_color_image_10);

                cv::Canny(xy_plane_image_0,xy_plane_image_0,10,30);
                cv::Canny(xy_plane_image_10,xy_plane_image_10,10,30);

//                cv::imwrite("xy_plane_image0.jpg",xy_plane_image_0);
//                cv::imwrite("xy_plane_image10.jpg",xy_plane_image_10);

                vector<cv::Vec4i> lines;
                vector<cv::Vec4i> lines2;

                // NEED TO BE TUNED THRESHOLD LEVEL FOR PANAL
                cv::HoughLinesP(xy_plane_image_0,lines,1,CV_PI/180,30,10,20);
                cv::HoughLinesP(xy_plane_image_10,lines2,1,CV_PI/180,30,10,20);

                double max_len =-1;
                double len =0;
                double max_len_slope =0;
                int max_len_index =0;
                cv::Vec4i l;

                double max_len2 =-1;
                double len2 =0;
                double max_len_slope2 =0;
                int max_len_index2 =0;
                cv::Vec4i l2;

                double parking_tr_x;
                double parking_tr_y;

                double line_dist =0;

                if(lines.size() != 0)
                {
                    for(int i=0; i<lines.size(); i++)
                    {
                        l = lines[i];
                        len = cv::norm(cv::Point(l[2],l[3]) - cv::Point(l[0],l[1]));
                        if(len > max_len)
                        {
                            max_len_index = i;
                            max_len = len;
                            max_len_slope = ((cv::Point(l[2],l[3]).y - cv::Point(l[0],l[1]).y) / (double)(cv::Point(l[2],l[3]).x - cv::Point(l[0],l[1]).x)) ;

                            parking_tr_x = 0.01*(((double)(cv::Point(l[2],l[3]).y + cv::Point(l[0],l[1]).y)/2.0) - 100.0*parking_distance*std::sin(max_len_slope - PI/2.0) - 300.0);
                            parking_tr_y = 0.01*(((double)(cv::Point(l[2],l[3]).x + cv::Point(l[0],l[1]).x)/2.0) + 100.0*parking_distance*std::cos(max_len_slope - PI/2.0) - 300.0);
                        }

                    }
                    cv::line(xy_plane_color_image_0,cv::Point(lines[max_len_index][0],(lines[max_len_index][1])),cv::Point((lines[max_len_index][2]),(lines[max_len_index][3])),cv::Scalar(0,0,255));
                }
                else
                    cout << "no line detected in angle 0 channel" << endl;


                if(lines2.size() != 0)
                {
                    for(int i=0; i<lines2.size(); i++)
                    {
                        l2 = lines2[i];
                        len2 = cv::norm(cv::Point(l2[2],l2[3]) - cv::Point(l2[0],l2[1]));
                        if(len2 > max_len2)
                        {
                            max_len_index2 = i;
                            max_len2 = len2;
                            max_len_slope2 = ((cv::Point(l2[2],l2[3]).y - cv::Point(l2[0],l2[1]).y) / (double)(cv::Point(l2[2],l2[3]).x - cv::Point(l2[0],l2[1]).x)) ;
                        }
                    }
                    cv::line(xy_plane_color_image_10,cv::Point(lines2[max_len_index2][0],(lines2[max_len_index2][1])),cv::Point((lines2[max_len_index2][2]),(lines2[max_len_index2][3])),cv::Scalar(0,255,0));
                }
                else
                    cout << "no line detected in angle 10 channel" << endl;

//                std::cout << "max len 0 : " << max_len << std::endl;
//                std::cout << "max len 10 : " << max_len2 << std::endl;

//                std::cout << "error : " << abs(max_len - max_len2) << endl;

//                cv::imwrite("xy_plane_image0.jpg",xy_plane_color_image_0);
//                cv::imwrite("xy_plane_image10.jpg",xy_plane_color_image_10);


//                if(lines2.size() != 0 && lines.size() != 0)
//                {
//                    cout << "angle 0's line rho and theta : " << lines[max_len_index][0] << "    " << lines[max_len_index][1] << endl;
//                    cout << "angle 10's line rho and theta : " << lines2[max_len_index2][0] << "    " << lines2[max_len_index2][1] << endl;

//                    line_dist = (cv::norm(cv::Point(lines2[max_len2][2],lines2[max_len2][3]) - cv::Point(lines[max_len][2],lines[max_len][3])) + cv::norm(cv::Point(lines2[max_len2][0],lines2[max_len2][1]) - cv::Point(lines[max_len][0],lines[max_len][1]))) / 2.0;

//                    cout  << "line1 theta : " << lines[max_len_index][0] << endl;
//                    cout  << "line2 theta : " << lines2[max_len_index2][0] << endl;


//                    cout  << "line1 b1 : " << lines[max_len_index][2]-lines[max_len_index][3]*max_len_slope << endl;
//                    cout  << "line2 b2 : " << lines2[max_len_index2][2]-lines2[max_len_index2][3]*max_len_slope2 << endl;

//                }

                const int WIDESIDE_MARGIN = 15;
                const int NARROWSIDE_MARGIN = 10;
                const int SLOPE_MARGIN = 1;

                const int WIDE_LEN = 100;
                const int SIDE_SHORT_LEN = 50;
                const int SIDE_LONG = 75;

//                cout << max_len << endl;


                if(max_len > WIDE_LEN - WIDESIDE_MARGIN && max_len < WIDE_LEN + WIDESIDE_MARGIN)
                {
                    if( abs(lines[max_len_index][3]-lines[max_len_index][2]*max_len_slope - lines2[max_len_index2][3]-lines2[max_len_index2][2]*max_len_slope2 ) > 80)
                    {
                        if( lines[max_len_index][3]-lines[max_len_index][2]*max_len_slope <= lines2[max_len_index2][3]-lines2[max_len_index2][2]*max_len_slope2 )
                        {
                            cout << "front side on the left" << endl;
                        }
                        else
                        {
                            cout << "front side on the right" << endl;
                        }
                    }
                    else
                    {
                        if(max_len_slope < -0.1)
                        {
                            cout << "back side on the left" << endl;
                        }
                        else if(max_len_slope > 0.1)
                        {
                            cout << "back side on the right" << endl;
                        }
                        else
                        {
                            cout << "back side" << endl;
                        }
                    }

                }
                else if( max_len > SIDE_SHORT_LEN - NARROWSIDE_MARGIN &&  max_len < SIDE_SHORT_LEN + NARROWSIDE_MARGIN)
                {
                    if( (lines2[max_len_index2][0] - lines[max_len_index][0]) + (lines2[max_len_index2][2] - lines[max_len_index][2])   > 10 )
                    {
                        cout << "left side" << endl;
                    }
                    else if ((lines2[max_len_index2][0] - lines[max_len_index][0]) + (lines2[max_len_index2][2] - lines[max_len_index][2]) < -10 )
                        cout << "right side" << endl;
                }
                else
                {
                    if(max_len == -1)
                        cout << "max len is not detected" << endl;
                    else
                        cout << "detected len is not for pannel range" << endl;
                }

                (*inliers).indices.clear();
                (*inliers2).indices.clear();

                mpc_pcl->viewer->updatePointCloud(mpc_pcl->cloud,"cloud");
//                mpc_pcl->viewer->updatePointCloud(ei_result,"cloud");
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

        QThread::usleep(30);


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








