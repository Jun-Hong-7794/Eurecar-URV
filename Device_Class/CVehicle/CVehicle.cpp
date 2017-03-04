#include "CVehicle.h"

CVehicle::CVehicle(){

    fl_isGO = false;
    fl_connection = false;

    m_vel = 0;
    m_timecnt = 0;
    m_dir = 0;
    m_status = -2;
    m_battery = 0;

    m_enc_left =0;
    m_enc_right =0;

    mstr_response = "";
}

CVehicle::~CVehicle(){

    if(fl_connection)
        mc_device.Disconnect();
}

bool CVehicle::IsConnected(){
    return fl_connection;
}

int CVehicle::Connect(char* _dev_path){

    if(!fl_connection){
        if(mc_device.Connect(_dev_path) == RQ_SUCCESS)
        {
            fl_connection = true;
            this->start();
        }
        else
            fl_connection = false;
    }

    return fl_connection;
}

bool CVehicle::InitEncoder()
{
    if(fl_connection){

        mc_device.SetCommand(UGV_DEF_C,1,0);
        mc_device.SetCommand(UGV_DEF_C,2,0);

        mc_device.SetCommand(UGV_DEF_GO,1,0);
        mc_device.SetCommand(UGV_DEF_GO,2,0);

        return true;
    }
    else
    {
        cout << " ugv connect first";
        return false;
    }
}

vector<int> CVehicle::GetEncoderValue()
{
//    vector<int> tmp;
//    int tmp_int;
//    int tmp_int2;
//    mc_device.GetValue(UGV_DEF_F,1,tmp_int);
//    mc_device.GetValue(UGV_DEF_F,2,tmp_int2);
//    tmp.push_back(abs(tmp_int));
//    tmp.push_back(abs(tmp_int2));
//    return tmp;

    vector<int> return_encoder_val = {0,0};
    mtx_vehicle.lock();
    return_encoder_val = {abs(m_enc_val.at(0)),abs(m_enc_val.at(1))};
    mtx_vehicle.unlock();
    return return_encoder_val;
}

vector<int> CVehicle::GetEncoderValueRaw()
{
//    vector<int> tmp;
//    int tmp_int;
//    int tmp_int2;
//    mc_device.GetValue(UGV_DEF_ABCNTR,1,tmp_int);
//    mc_device.GetValue(UGV_DEF_ABCNTR,2,tmp_int2);
//    tmp.push_back(tmp_int);
//    tmp.push_back(tmp_int2);
//    return tmp;
//    while(m_enc_val.size() != 2) {};

    vector<int> return_encoder_val = {0,0};
    mtx_vehicle.lock();
    return_encoder_val = m_enc_val;
    mtx_vehicle.unlock();
    return m_enc_val;
}

int CVehicle::CalcDistToEnc_m(double _dist_meter)
{
    double coeff[2] = {93.1531, -28.2715};
    double tmp = _dist_meter*coeff[0] + coeff[1];

    return (int)(tmp);
}
void CVehicle::Disconnect(){

    mc_device.Disconnect();
    fl_connection = false;
}

bool CVehicle::Move(int _dir, int _vel){

    int control_target_velocity = _vel;
    int rst__left_side = -1;
    int rst_right_side = -1;

    cout << "current velocity : " << _vel << endl;

    // Check point start for vehicle turning availability ------------------

    gettimeofday(&m_cur_time,NULL);

    long time_lapsed = 0;
    double heading_changed = 0.0;

    if((_dir == m_past_direction_command) && ((_dir == UGV_move_left) || (_dir == UGV_move_right) || (_dir == UGV_move_differ_left)))
    {
        if(m_reference_timestamp == 0)
        {
            m_reference_timestamp = m_cur_time.tv_sec*1000 + (double)m_cur_time.tv_usec/1000.0;
            m_reference_heading = m_current_heading;
        }
        else
        {
            m_current_timestamp = m_cur_time.tv_sec*1000 + (double)m_cur_time.tv_usec/1000.0;
            time_lapsed = m_current_timestamp - m_reference_timestamp;
            heading_changed = abs(m_current_heading - m_reference_heading)/3.1415926535*180.0;
            if( (time_lapsed > 1000) && (heading_changed < 20.0))
            {
                int vel_increase_scale_factor = time_lapsed/1000 + 1;

                control_target_velocity = _vel + 10*vel_increase_scale_factor;
            }

            // velocity increase saturation
            if(control_target_velocity > 150)
            {
                control_target_velocity = 150;
            }
        }
    }
    else
    {
        m_reference_timestamp = 0;
    }
    // ---------------------------------------------------------------------

    switch (_dir) {
    case UGV_move_forward:
        m_vel = control_target_velocity;
        m_dir = UGV_move_forward;
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(1.1));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(-1.1));
        turn_continue_flag_left =0;
        turn_continue_flag_right=0;
        break;
    case UGV_move_backward:
        m_vel = control_target_velocity;
        m_dir = UGV_move_backward;
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(1.1));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(-1.1));
        turn_continue_flag_left =0;
        turn_continue_flag_right=0;
        break;
    case UGV_move_left:
        m_vel = control_target_velocity;
        m_dir = UGV_move_left;
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(-2.2));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(-2.2));
        turn_continue_flag_left =1;
        turn_continue_flag_right=0;
        break;
    case UGV_move_right:
        m_vel = control_target_velocity;
        m_dir = UGV_move_right;
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(2.2));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(2.2));
        turn_continue_flag_left =0;
        turn_continue_flag_right=1;
        break;
    case UGV_move_differ_left:
        m_vel = control_target_velocity;
        m_dir = UGV_move_differ_left;
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(-2.2));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(-3));
        turn_continue_flag_left =0;
        turn_continue_flag_right=0;
        break;
    case UGV_move_differ_right:
        m_vel = control_target_velocity;
        m_dir = UGV_move_differ_right;
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(2.2));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(3));
        turn_continue_flag_left =0;
        turn_continue_flag_right=0;
        break;
    default:
        break;
    }

    if(rst__left_side != RQ_SUCCESS){
        std::cout << "Fail to Set Command on Left Side" << std::endl;
        return false;
    }

    if(rst_right_side != RQ_SUCCESS){
        std::cout << "Fail to Set Command on Right Side" << std::endl;
        return false;
    }

    sleepms(30);

    return true;
}

//bool CVehicle::Move(int _dir, int _vel) { // use double velocity

//    int control_target_velocity = _vel;
//    int rst__left_side = -1;
//    int rst_right_side = -1;

//    cout << "double velocity is used" << endl;

//    switch (_dir) {
//    case UGV_move_forward:
//        m_vel = control_target_velocity;
//        m_dir = UGV_move_forward;
//        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(1.1));
//        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(-1.1));
//        turn_continue_flag_left =0;
//        turn_continue_flag_right=0;
//        break;
//    case UGV_move_backward:
//        m_vel = control_target_velocity;
//        m_dir = UGV_move_backward;
//        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(1.1));
//        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(-1.1));
//        turn_continue_flag_left =0;
//        turn_continue_flag_right=0;
//        break;
//    case UGV_move_left:
//        m_vel = control_target_velocity;
//        m_dir = UGV_move_left;
//        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(-2.2));
//        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(-2.2));
//        turn_continue_flag_left =1;
//        turn_continue_flag_right=0;
//        break;
//    case UGV_move_right:
//        m_vel = control_target_velocity;
//        m_dir = UGV_move_right;
//        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(2.2));
//        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(2.2));
//        turn_continue_flag_left =0;
//        turn_continue_flag_right=1;
//        break;
//    case UGV_move_differ_left:
//        m_vel = control_target_velocity;
//        m_dir = UGV_move_differ_left;
//        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(-1.8));
//        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(-3));
//        turn_continue_flag_left =0;
//        turn_continue_flag_right=0;
//        break;
//    case UGV_move_differ_right:
//        m_vel = control_target_velocity;
//        m_dir = UGV_move_differ_right;
//        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,m_vel*(1.8));
//        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,m_vel*(3));
//        turn_continue_flag_left =0;
//        turn_continue_flag_right=0;
//        break;
//    default:
//        break;
//    }

//    if(rst__left_side != RQ_SUCCESS){
//        std::cout << "Fail to Set Command on Left Side" << std::endl;
//        return false;
//    }

//    if(rst_right_side != RQ_SUCCESS){
//        std::cout << "Fail to Set Command on Right Side" << std::endl;
//        return false;
//    }

//    sleepms(30);

//    cout << "m_vel : " << m_vel << endl;

//    return true;
//}


bool CVehicle::ActiveMagnet(bool _on_off){

    int command_rst = 0;

    if(_on_off){
        command_rst = mc_device.SetCommand(UGV_DEF_DRES, 6);
        mc_device.SetCommand(UGV_DEF_GO,1,0);
        mc_device.SetCommand(UGV_DEF_GO,2,0);
    }
    else{
        command_rst = mc_device.SetCommand(UGV_DEF_DSET, 6);
        mc_device.SetCommand(UGV_DEF_GO,1,0);
        mc_device.SetCommand(UGV_DEF_GO,2,0);
    }

    if(command_rst != RQ_SUCCESS){
        std::cout << "Fail to Set Command Active Magnet" << std::endl;
        return false;
    }

    return true;
}


void CVehicle::CheckVolt(){
    int main_bat = 2;
    m_battery = mc_device.GetValue(UGV_DEF_VOLTS,main_bat);
}

int CVehicle::GetVel()
{
    return m_vel;
}

void CVehicle::CheckEncoderValue()
{
    mtx_vehicle.lock();
    mc_device.GetValue(UGV_DEF_ABCNTR,1,m_enc_right);
    mc_device.GetValue(UGV_DEF_ABCNTR,2,m_enc_left);
    m_enc_val.clear();
    m_enc_val.push_back(m_enc_right);
    m_enc_val.push_back(m_enc_left);
    mtx_vehicle.unlock();
}

void CVehicle::SlotVehicleHeading(double _heading)
{
    m_current_heading = _heading;
}

//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CVehicle::run()
{
//    while(fl_isGO){
//        Move(m_dir,m_vel);
//        sleepms(20);
//    }

    while(fl_connection)
    {
        CheckEncoderValue();
        sleepms(20);
    }
}
