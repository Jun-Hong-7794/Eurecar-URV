#include "CLRF.h"

CLRF::CLRF()
{
    fl_stream = false;
    fl_lrf_init = false;

    m_number_of_point = 0;

    mp_connect_address = new char[20];
    mp_connect_address = (char *)"192.168.0.10";
    m_connect_port = 10940;

    mp_connect_device  = new char[20];
    mp_connect_device = (char *)"/dev/ttyACM0";
    m_baudrate = 115200;
}

CLRF::~CLRF(){

    CloseLRF();

}

bool CLRF::LRFOpen(int _dev_type){

    int ret = 0;

    if(_dev_type == UST_20LX){
        mtx_LRF_data.lock();
        {
            ret = urg_open(&m_urg, URG_ETHERNET, mp_connect_address, m_connect_port);

            //_data  = new long[urg_max_data_size(&m_urg)];

            mp_distance_data = new long[urg_max_data_size(&m_urg)];
            mp_intensity_data = new unsigned short[urg_max_data_size(&m_urg)];
        }
        mtx_LRF_data.unlock();

        // Check error code
        if(ret < 0){
            std::cout << "Fail to Open Device"<< std::endl;
            std::cout << "Device IP : "<< mp_connect_address << std::endl;
            std::cout << "Device Port : "<< m_connect_port << std::endl;
            return false;
        }
        else{
            std::cout << "Success to Open Device"<< std::endl;
        }

        mtx_LRF_data.lock();
        {
            ret = urg_start_measurement(&m_urg, URG_DISTANCE_INTENSITY, URG_SCAN_INFINITY, 0);
            //ret = urg_start_measurement(&m_urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);
        }
        mtx_LRF_data.unlock();

        m_device_type = UST_20LX;
    }
    else{//UTM_30LX
        mtx_LRF_data.lock();
        {
            ret = urg_open(&m_urg, URG_SERIAL, mp_connect_device, m_baudrate);

            // Check error code
            if(ret < 0){
                std::cout << "Fail to Open Device"<< std::endl;
                std::cout << "Device : "<< mp_connect_device << std::endl;
                std::cout << "Device Baudrate: "<< m_baudrate << std::endl;
                return false;
            }
            else{
                std::cout << "Success to Open Device"<< std::endl;
            }

            mp_distance_data = new long[urg_max_data_size(&m_urg)];

            ret = urg_start_measurement(&m_urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);

            if(ret < 0){
                std::cout << "Fail to Start Device"<< std::endl;
                std::cout << "Device : "<< mp_connect_device << std::endl;
                std::cout << "Device Baudrate: "<< m_baudrate << std::endl;
                return false;
            }
            else{
                std::cout << "Success to Start Device"<< std::endl;
            }

        }
        mtx_LRF_data.unlock();

        m_device_type = UTM_30LX;
    }

    fl_stream = true;

    this->start();

    return true;
}

bool CLRF::InitLRF(char* _dev_path, int _dev_type){

    if(fl_lrf_init)
        return true;

    mp_connect_device = _dev_path;

    if(!LRFOpen(_dev_type))
        return false;

    else{
        fl_lrf_init = true;
        return true;
    }
}

void CLRF::CloseLRF(){

    fl_stream = false;
    fl_lrf_init = false;

    while(this->isRunning());

    mtx_LRF_data.lock();
    {
        urg_close(&m_urg);
    }
    mtx_LRF_data.unlock();
}

bool CLRF::IsLRFOn(){
    return fl_stream;
}

int CLRF::GetDeviceType(){
    return m_device_type;
}

bool CLRF::CalculateColorStep(unsigned int _max_num,unsigned int _current_num, int &_red,int &_green,int &_blue){

    double color_step = ((double)_current_num / (double)_max_num);

    if(color_step > 1){
        color_step = 1;
    }

    color_step *= (255*3);

    _blue = color_step - 510;
    _green = color_step - 255;
    _red = color_step - 0;

    if(_blue > 255)
        _blue = 255;
    if(_blue < 0)
        _blue = 0;

    if(_green > 255)
        _green = 255;
    if(_green < 0)
        _green = 0;

    if(_red > 255)
        _red = 255;
    if(_red < 0)
        _red = 255;

    return true;
}

cv::Mat CLRF::LRFDataToMat(int _s_index, int _e_index){

    cv::Mat lrf_map = cv::Mat::zeros(480,640,CV_8UC3);

    long* lrf_distance_data = NULL;

    int width = lrf_map.cols * 3;
    int height = lrf_map.rows;

    if(width == 0 || height == 0){
        return lrf_map;
    }

    mtx_LRF_data.lock();
    {
        lrf_distance_data = new long[urg_max_data_size(&m_urg)];
        memcpy(lrf_distance_data,mp_distance_data,sizeof(long) * urg_max_data_size(&m_urg));
    }
    mtx_LRF_data.unlock();

    for(int i = _s_index; i < _e_index; i++){
        double radian;
        long length;
        int x;
        int y;

        radian = urg_index2rad(&m_urg, i);
        length = lrf_distance_data[i];

        x = (int)( ((length * cos(radian) * 240) / (MAX_LRF_RANGE)) + 320);
        y = (int)( ((length * sin((-1)*radian) * 240) / (MAX_LRF_RANGE)) + 240);

        if(x >= 640) x = 0;
        if(x <    0) x = 0;

        if(y >= 480) y = 0;
        if(y <    0) y = 0;

        int red = 0;
        int green = 0;
        int blue = 0;

        CalculateColorStep(2000,length,red,green,blue);
        lrf_map.data[y*width + x*3 + 0] = blue; //B
        lrf_map.data[y*width + x*3 + 1] = green; //G
        lrf_map.data[y*width + x*3 + 2] = red; //R
    }

    return lrf_map;
}

void CLRF::LRFDataToMat(cv::Mat &_lrf_map){

    int width = _lrf_map.cols * 3;
    int height = _lrf_map.rows;

    long* lrf_distance_data = NULL;

    if(width == 0 || height == 0){
        return;
    }

    mtx_LRF_data.lock();
    {
        lrf_distance_data = new long[urg_max_data_size(&m_urg)];
        memcpy(lrf_distance_data,mp_distance_data,sizeof(long) * urg_max_data_size(&m_urg));
    }
    mtx_LRF_data.unlock();

    for(int i = 0; i < m_number_of_point; i++){
        double radian;
        long length;
        int x;
        int y;

        radian = urg_index2rad(&m_urg, i);
        length = lrf_distance_data[i];

        x = (int)( ((length * cos(radian) * 240) / (MAX_LRF_RANGE)) + 320);
        y = (int)( ((length * sin((-1)*radian) * 240) / (MAX_LRF_RANGE)) + 240);

        if(x >= 640) x = 0;
        if(x <    0) x = 0;

        if(y >= 480) y = 0;
        if(y <    0) y = 0;

        int red = 0;
        int green = 0;
        int blue = 0;

        CalculateColorStep(2000,length,red,green,blue);
        _lrf_map.data[y*width + x*3 + 0] = blue; //B
        _lrf_map.data[y*width + x*3 + 1] = green; //G
        _lrf_map.data[y*width + x*3 + 2] = red; //R
    }

    delete[] lrf_distance_data;
}

bool CLRF::GetLRFData(long *_distance_data, unsigned short *_intensity_data){


    if(!fl_lrf_init)
        return false;

    mtx_LRF_data.lock();
    {
        memcpy(_distance_data,mp_distance_data,sizeof(long) * urg_max_data_size(&m_urg));

        if(m_device_type == UST_20LX)
            memcpy(_intensity_data,mp_intensity_data,sizeof(unsigned short) * urg_max_data_size(&m_urg));

    }
    mtx_LRF_data.unlock();

    return true;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CLRF::run(){

    while(fl_stream){

       cv::Mat lrf_mat = cv::Mat::zeros(480,640,CV_8UC3);

        mtx_LRF_data.lock();
        {

            if(m_device_type == UST_20LX)
                m_number_of_point = urg_get_distance_intensity(&m_urg, mp_distance_data, mp_intensity_data, NULL);
            else//UTM_30LX
                m_number_of_point = urg_get_distance(&m_urg, mp_distance_data, NULL);
        }
        mtx_LRF_data.unlock();

//        LRFDataToMat(lrf_mat);
//        lrf_mat = LRFDataToMat(75*4,195*4);

//        emit SignalLRFImage(lrf_mat);

        lrf_mat.release();
    }
}
