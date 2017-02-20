#include "CVehicle.h"

CVehicle::CVehicle(){

    fl_isGO = false;
    fl_connection = false;

    m_vel = 0;
    m_timecnt = 0;
    m_dir = 0;
    m_status = -2;
    m_battery = 0;

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
            fl_connection = true;
        else
            fl_connection = false;
    }

    return fl_connection;
}

void CVehicle::Disconnect(){

    mc_device.Disconnect();
    fl_connection = false;
}

bool CVehicle::Move(int _dir, int _vel){

    int rst__left_side = -1;
    int rst_right_side = -1;

    switch (_dir) {
    case UGV_move_forward:
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,_vel*(1.1));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,_vel*(-1.1));
        break;
    case UGV_move_backward:
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,2,_vel*(1.1));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,1,_vel*(-1.1));
        break;
    case UGV_move_left:
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,_vel*(-2.2));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,_vel*(-2.2));
        break;
    case UGV_move_right:
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,_vel*(2.2));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,_vel*(2.2));
        break;
    case UGV_move_differ_left:
        rst__left_side = mc_device.SetCommand(UGV_DEF_GO,1,_vel*(-1.8));
        rst_right_side = mc_device.SetCommand(UGV_DEF_GO,2,_vel*(-3));
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

    sleepms(20);

    return true;
}

bool CVehicle::ActiveMagnet(bool _on_off){

    int command_rst = 0;

    if(_on_off){
        command_rst = mc_device.SetCommand(UGV_DEF_DRES, 6);
    }
    else{
        command_rst = mc_device.SetCommand(UGV_DEF_DSET, 6);
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


//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CVehicle::run()
{
    while(fl_isGO){
        Move(m_dir,m_vel);
        sleepms(20);
    }
}
