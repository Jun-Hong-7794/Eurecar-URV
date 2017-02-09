#include "CGripper.h"

CGripper::CGripper()
{
    fl_init_dynamixel = false;
    fl_torque = false;
}

CGripper::~CGripper()
{
    CloseDynamixel();
}


bool CGripper::InitDynamixel(char* _device_port){

    if(fl_init_dynamixel)
        return true;

    mtx_dmx_handle.lock();
    {
        mp_portHandler = dynamixel::PortHandler::getPortHandler(_device_port);
        mp_packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open port
        if (mp_portHandler->openPort()){
            printf("Succeeded to open the port!\n");
        }
        else{
            printf("Failed to open the port!\n");
            //getchar();
            mtx_dmx_handle.unlock();
            return false;
        }

        // Set port baudrate
        if (mp_portHandler->setBaudRate(BAUDRATE)){
            printf("Succeeded to change the baudrate!\n");
        }
        else{
            printf("Failed to change the baudrate!\n");
            mtx_dmx_handle.unlock();
            return false;
        }
        dxl_comm_result = mp_packetHandler->write2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_GOAL_SPEED, DXL_GOAL_VELOCITY_VALUE, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS){
            mp_packetHandler->printTxRxResult(dxl_comm_result);
            mtx_dmx_handle.unlock();
            return false;
        }
        else if (dxl_error != 0){
            mp_packetHandler->printRxPacketError(dxl_error);
            mtx_dmx_handle.unlock();
            return false;
        }
    }
    mtx_dmx_handle.unlock();

    fl_init_dynamixel = true;

    return true;
}

void CGripper::CloseDynamixel(){

    if(!fl_init_dynamixel)
        return;

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    mtx_dmx_handle.lock();
    {
        // Disable Dynamixel Torque
        dxl_comm_result = mp_packetHandler->write1ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            mp_packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
            mp_packetHandler->printRxPacketError(dxl_error);
        }

        // Close port
        mp_portHandler->closePort();
    }
    mtx_dmx_handle.unlock();

    fl_init_dynamixel = false;
    printf("Close the Port!\n");

}

bool CGripper::DynamixelTorque(bool _onoff){

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    mtx_dmx_handle.lock();
    {
        dxl_comm_result = mp_packetHandler->write1ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, _onoff, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS){
            mp_packetHandler->printTxRxResult(dxl_comm_result);
            printf("Enable Dynamixel Torque : Failed to COMM_SUCCESS\n");
            mtx_dmx_handle.unlock();
            return false;
        }
        else if (dxl_error != 0){
            mp_packetHandler->printRxPacketError(dxl_error);
            printf("Enable Dynamixel Torque : dxl_error\n");
            mtx_dmx_handle.unlock();
            return false;
        }
        else{
            printf("Dynamixel has been successfully connected \n");
        }
    }
    mtx_dmx_handle.unlock();

    if(_onoff)
        fl_torque = true;
    else
        fl_torque = false;

    return true;
}

bool CGripper::DynamixelGoToThePosition(int _degree){

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int dxl_goal_position = (int)(_degree * DXL_STEP_PER_DEGREE);  // Goal position
    uint16_t dxl_present_position = 0; // Present position

    mtx_dmx_handle.lock();
    {
        // Write goal position
        dxl_comm_result = mp_packetHandler->write2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            mp_packetHandler->printTxRxResult(dxl_comm_result);
            printf("Write goal position : Failed to COMM_SUCCESS\n");
            mtx_dmx_handle.unlock();
            return false;
        }
        else if (dxl_error != 0){
            mp_packetHandler->printRxPacketError(dxl_error);
            printf("Write goal position : dxl_error\n");
            mtx_dmx_handle.unlock();
            return false;
        }

        do
        {
            // Read present position
            dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS){
                mp_packetHandler->printTxRxResult(dxl_comm_result);
                printf("Read present position : Failed to COMM_SUCCESS\n");
            }
            else if (dxl_error != 0){
                mp_packetHandler->printRxPacketError(dxl_error);
                printf("Read present position : dxl_error\n");
            }

            //        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position, dxl_present_position);

        }while((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }
    mtx_dmx_handle.unlock();

    return true;
}

bool CGripper::DynamixelGoToThePositionUsingLoad(int _degree, int _load_threshold_up, int _load_threshold_dw){ // Go to The Position

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int dxl_step = 1;
    int dxl_goal_position = (int)(_degree * DXL_STEP_PER_DEGREE);  // Goal position
    int dxl_next_position = 0;
    uint16_t dxl_prev_position = 0;
    uint16_t dxl_present_position = 0; // Present position
    uint16_t dxl_present_load = 0; // Present position

    mtx_dmx_handle.lock();
    {
        dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        dxl_prev_position = dxl_present_position;

        if((dxl_goal_position - dxl_present_position) > 0)
            dxl_step = 1;
        else
            dxl_step = -1;

        do
        {
            // Read present position
            dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);

            if(_load_threshold_up != 0){// if 0, do not use force ctrl_load_threshold_up
                if((dxl_present_load > _load_threshold_up) && (dxl_present_load < _load_threshold_dw)){
                    std::cout << "Force Occured!" << std::endl;
                    std::cout << "Present Gripper Load :" << dxl_present_load << std::endl;
                    std::cout << "Present Goal Position :" << dxl_present_position << std::endl;
                    mtx_dmx_handle.unlock();
                    return false;
                }
            }

            dxl_next_position = dxl_prev_position + dxl_step;
            dxl_comm_result = mp_packetHandler->write2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_next_position, &dxl_error);


            if (dxl_comm_result != COMM_SUCCESS){
                mp_packetHandler->printTxRxResult(dxl_comm_result);
                printf("Read present position : Failed to COMM_SUCCESS\n");
            }
            else if (dxl_error != 0){
                mp_packetHandler->printRxPacketError(dxl_error);
                printf("Read present position : dxl_error\n");
            }

            dxl_prev_position = dxl_next_position;
        }while((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }

    mtx_dmx_handle.unlock();

    return true;
}

bool CGripper::DynamixelGoToRelPosition(double _degree){

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int dxl_goal_position;  // Goal position
    uint16_t dxl_present_position = 0; // Present position

    mtx_dmx_handle.lock();
    {
        // Read present position
        dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS){
            mp_packetHandler->printTxRxResult(dxl_comm_result);
            printf("Read present position : Failed\n");
            mtx_dmx_handle.unlock();
            return false;
        }
        else if (dxl_error != 0){
            mp_packetHandler->printRxPacketError(dxl_error);
            printf("Read present position : dxl_error\n");
            mtx_dmx_handle.unlock();
            return false;
        }
        else{
            dxl_goal_position = (int)(_degree * DXL_STEP_PER_DEGREE) + dxl_present_position;
        }

        // Write goal position
        dxl_comm_result = mp_packetHandler->write2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            mp_packetHandler->printTxRxResult(dxl_comm_result);
            printf("Write goal position : Failed to COMM_SUCCESS\n");
            mtx_dmx_handle.unlock();
            return false;
        }
        else if (dxl_error != 0){
            mp_packetHandler->printRxPacketError(dxl_error);
            printf("Write goal position : dxl_error\n");
            mtx_dmx_handle.unlock();
            return false;
        }

        do
        {
            // Read present position
            dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS){
                mp_packetHandler->printTxRxResult(dxl_comm_result);
                printf("Read present position : Failed to COMM_SUCCESS\n");
                mtx_dmx_handle.unlock();
                return false;
            }
            else if (dxl_error != 0){
                mp_packetHandler->printRxPacketError(dxl_error);
                printf("Read present position : dxl_error\n");
                mtx_dmx_handle.unlock();
                return false;
            }

        }while((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }
    mtx_dmx_handle.unlock();

    return true;
}


bool CGripper::IsDmxInit(){

    if(!InitDynamixel())
        return false;

    return true;
}

bool CGripper::IsDmxTorqueOn(){

    if(!DynamixelTorque(true))
        return false;

    return true;
}

uint16_t CGripper::DynamixelPresentPosition(){

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    uint16_t dxl_present_position = 0; // Present position

    mtx_dmx_handle.lock();
    {
        // Read present position
        dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS){
            mp_packetHandler->printTxRxResult(dxl_comm_result);
            printf("Read present position : Failed to COMM_SUCCESS\n");
            mtx_dmx_handle.unlock();
            return -1;
        }
        else if (dxl_error != 0){
            mp_packetHandler->printRxPacketError(dxl_error);
            printf("Read present position : dxl_error\n");
            mtx_dmx_handle.unlock();
            return -1;
        }
    }
    mtx_dmx_handle.unlock();
    return dxl_present_position;
}

uint16_t CGripper::DynamixelPresentLoad(){

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    uint16_t dxl_present_load = 0; // Present Load

    mtx_dmx_handle.lock();
    {
        // Read present position
        dxl_comm_result = mp_packetHandler->read2ByteTxRx(mp_portHandler, DXL_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS){
            mp_packetHandler->printTxRxResult(dxl_comm_result);
            printf("Read present load : Failed\n");
            mtx_dmx_handle.unlock();
            return -1;
        }
        else if (dxl_error != 0){
            mp_packetHandler->printRxPacketError(dxl_error);
            printf("Read present load : dxl_error\n");
            mtx_dmx_handle.unlock();
            return -1;
        }
    }
    mtx_dmx_handle.unlock();
    return dxl_present_load;
}

//----------------------------------------------------------------
//
//                            Gripper
//
//----------------------------------------------------------------


bool CGripper::IsGripperInit(){
    if(!fl_init_gripper) return false;
    return true;
}


bool CGripper::IsGripperTorqueOn(){
    if(!fl_torque_gripper) return false;
    return true;
}

bool CGripper::InitGripper(char* _device_port){

    mp_gripper_portHandler = dynamixel::PortHandler::getPortHandler(_device_port);
    mp_gripper_packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    mp_groupBulkRead_pos = new dynamixel::GroupBulkRead(mp_gripper_portHandler, mp_gripper_packetHandler);
    mp_groupBulkRead_tor = new dynamixel::GroupBulkRead(mp_gripper_portHandler, mp_gripper_packetHandler);

    // Open port
    if (mp_gripper_portHandler->openPort()){
        printf("Succeeded to open the port!\n");
    }
    else{
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getchar();
        return false;
    }

    // Set port baudrate
    if (mp_gripper_portHandler->setBaudRate(BAUDRATE)){
        printf("Succeeded to change the baudrate!\n");
    }
    else{
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getchar();
        return false;
    }

    fl_torque_gripper = false;
    GripperTorque(true);
    dxl_goal_position = DXL_INITIAL_POSITION_VALUE;

    dxl_addparam_result = mp_groupBulkRead_pos->addParam(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position addparam failed\n", DXL1_ID);
        return false;
    }
    dxl_addparam_result = mp_groupBulkRead_pos->addParam(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position addparam failed\n", DXL2_ID);
        return false;
    }
    dxl_addparam_result = mp_groupBulkRead_tor->addParam(DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present load addparam failed\n", DXL1_ID);
        return false;
    }
    dxl_addparam_result = mp_groupBulkRead_tor->addParam(DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present load addparam failed\n", DXL2_ID);
        return false;
    }

    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_SPEED, DXL_GOAL_VELOCITY_VALUE, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0) mp_gripper_packetHandler->printRxPacketError(dxl_error);
    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_SPEED, DXL_GOAL_VELOCITY_VALUE, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0) mp_gripper_packetHandler->printRxPacketError(dxl_error);

    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position/*+DXL1_OFFSET*/, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_gripper_packetHandler->printRxPacketError(dxl_error);

    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_gripper_packetHandler->printRxPacketError(dxl_error);

    fl_init_gripper = true;

    dxl_comm_result = mp_groupBulkRead_pos->txRxPacket();
    if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    dxl_getdata_result = mp_groupBulkRead_pos->isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    if(!dxl_getdata_result) fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position getdata failed", DXL1_ID);
    dxl_getdata_result = mp_groupBulkRead_pos->isAvailable(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    if(!dxl_getdata_result) fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position getdata failed", DXL2_ID);
    dxl1_present_position = mp_groupBulkRead_pos->getData(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    dxl2_present_position = mp_groupBulkRead_pos->getData(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    return true;
}

void CGripper::CloseGripper(){
    GripperTorque(false);
    mp_gripper_portHandler->closePort();
    fl_init_gripper= false;
}

bool CGripper::GripperTorque(bool _onoff){

    if(_onoff){
        if(fl_torque_gripper) return false;
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) mp_gripper_packetHandler->printTxRxResult(dxl_error);
        else printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) mp_gripper_packetHandler->printTxRxResult(dxl_error);
        else printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
        fl_torque_gripper = true;
    }
    else{
        if(!fl_torque_gripper) return false;
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) mp_gripper_packetHandler->printTxRxResult(dxl_error);
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) mp_gripper_packetHandler->printTxRxResult(dxl_error);
        fl_torque_gripper = false;
    }
    return true;
}

bool CGripper::GripperGoToThePosition(int _degree){ // Go to The Position

    dynamixel::GroupBulkRead groupBulkRead_pos(mp_gripper_portHandler, mp_gripper_packetHandler);
    dynamixel::GroupBulkRead groupBulkRead_tor(mp_gripper_portHandler, mp_gripper_packetHandler);

    dxl_comm_result = mp_groupBulkRead_pos->txRxPacket();

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

    dxl_getdata_result = mp_groupBulkRead_pos->isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    if(!dxl_getdata_result)
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position getdata failed", DXL1_ID);

    dxl_getdata_result = mp_groupBulkRead_pos->isAvailable(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    if(!dxl_getdata_result)
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position getdata failed", DXL2_ID);

    dxl1_present_position = mp_groupBulkRead_pos->getData(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//    dxl1_present_position -= DXL1_OFFSET;
    dxl2_present_position = mp_groupBulkRead_pos->getData(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    int i = 0;
//    int previous = 0;
    int cnt = 0;

    if(_degree >= (dxl1_present_position+dxl2_present_position)/2){// When Release Gripper....
        dxl_goal_position = _degree;
        dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position+DXL1_OFFSET, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printRxPacketError(dxl_error);

        dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printRxPacketError(dxl_error);
    }
    else{// When Grasp....
        while(true){//To use force feed back, move dynamixel step by step

//            previous = (dxl1_present_position + dxl2_present_position)/2;
            sleep(0.5);
            if(dxl_goal_position > _degree){
                i++;
                dxl_goal_position = (dxl1_present_position+dxl2_present_position)/2 - 5*i;
            }

            dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position/*+DXL1_OFFSET*/, &dxl_error);

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            else if(dxl_error != 0)
                mp_gripper_packetHandler->printRxPacketError(dxl_error);

            dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            else if(dxl_error != 0)
                mp_gripper_packetHandler->printRxPacketError(dxl_error);

            dxl_comm_result = mp_groupBulkRead_pos->txRxPacket();

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            dxl_getdata_result = mp_groupBulkRead_pos->isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position getdata failed", DXL1_ID);

            dxl_getdata_result = mp_groupBulkRead_pos->isAvailable(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position getdata failed", DXL2_ID);

            dxl1_present_position = mp_groupBulkRead_pos->getData(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
//            dxl1_present_position -= DXL1_OFFSET;
            dxl2_present_position = mp_groupBulkRead_pos->getData(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

            dxl_comm_result = mp_groupBulkRead_tor->txRxPacket();

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            dxl_getdata_result = mp_groupBulkRead_tor->isAvailable(DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present load getdata failed", DXL1_ID);

            dxl_getdata_result = mp_groupBulkRead_tor->isAvailable(DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present load getdata failed", DXL2_ID);

            dxl1_present_load= mp_groupBulkRead_tor->getData(DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);
            dxl2_present_load= mp_groupBulkRead_tor->getData(DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);

            std::cout<<"dxl1 present load: " << dxl1_present_load<< "\t dxl2 present load: " << dxl2_present_load<< std::endl;
            std::cout<<"dxl1 present angle: " << dxl1_present_position << "\t dxl2 present angle : " << dxl2_present_position << std::endl;

            if(((dxl1_present_load > 180)&&(dxl1_present_load < 1023))||((dxl2_present_load > 300)&&(dxl2_present_load < 1023)))
                break;

            if((dxl1_present_position + dxl2_present_position)/2 - _degree <= 10) break;

            else if(((dxl1_present_position+dxl2_present_position)/2 - _degree > 10)&&((dxl1_present_position+dxl2_present_position)/2 - _degree <= 25)){
                cnt++;
                if(cnt > 100) break;
            }
        }
    }
    return true;
}

bool CGripper::GripperGoToThePositionLoadCheck(int _goal_pos_1, int _goal_pos_2, int _load_threshold){ // Go to The Position

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int dxl_step_1 = 1;
    int dxl_next_position_1 = 0;
    uint16_t dxl_prev_position_1 = 0;
    uint16_t dxl_present_position_1 = 0; // Present position
    uint16_t dxl_present_load_1 = 0; // Present position

    int dxl_step_2 = 1;
    int dxl_next_position_2 = 0;
    uint16_t dxl_prev_position_2 = 0;
    uint16_t dxl_present_position_2 = 0; // Present position
    uint16_t dxl_present_load_2 = 0; // Present position

    GRIPPER_STATUS gripper_status;

    mtx_gripper_handle.lock();
    {
        dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position_1, &dxl_error);
        dxl_prev_position_1 = dxl_present_position_1;

        dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position_2, &dxl_error);
        dxl_prev_position_2 = dxl_present_position_2;

        if(_load_threshold == -2){// Go to the position no load and no unit step

            if(_goal_pos_1 > 0){
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, _goal_pos_1, &dxl_error);

                if(dxl_comm_result != COMM_SUCCESS){
                    mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                    mtx_gripper_handle.unlock();
                    return false;
                }
                else if(dxl_error != 0)
                    mp_gripper_packetHandler->printRxPacketError(dxl_error);
            }
            if(_goal_pos_2 > 0){
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, _goal_pos_2, &dxl_error);

                if(dxl_comm_result != COMM_SUCCESS){
                    mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                    mtx_gripper_handle.unlock();
                    return false;
                }
                else if(dxl_error != 0)
                    mp_gripper_packetHandler->printRxPacketError(dxl_error);
            }

            mtx_gripper_handle.unlock();
            return true;
        }

        if((_goal_pos_1 - dxl_present_position_1) > 0)
            dxl_step_1 = 1;
        else
            dxl_step_1 = -1;

        if((_goal_pos_2 - dxl_present_position_2) > 0)
            dxl_step_2 = 1;
        else
            dxl_step_2 = -1;

        do
        {
            if(_goal_pos_1 != -1){
                // Read present position
                dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position_1, &dxl_error);
                dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load_1, &dxl_error);
            }
            if(_goal_pos_2 != -1){
                // Read present position
                dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position_2, &dxl_error);
                dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load_2, &dxl_error);
            }

            gripper_status.present_pose_1 = dxl_present_position_1;
            gripper_status.present_pose_2 = dxl_present_position_2;

            gripper_status.present_load_1 = dxl_present_load_1;
            gripper_status.present_load_2 = dxl_present_load_2;

            emit SignalEditeGripperStatus(gripper_status);

            if(_load_threshold != 0){// if 0, do not use force ctrl_load_threshold

                if(_goal_pos_1 != -1){
                    if(dxl_step_1 == 1){
                        if((dxl_present_load_1 > 1200)){
                            std::cout << "Force Occured!" << std::endl;
                            std::cout << "Present Gripper Load :" << dxl_present_load_1 << std::endl;
                            std::cout << "Present Goal Position :" << dxl_present_position_1 << std::endl;
                            _goal_pos_1 = -1;
                        }
                    }
                    else{
                        if((dxl_present_load_1> _load_threshold) && (dxl_present_load_1 < 1000)){
                            std::cout << "Force Occured!" << std::endl;
                            std::cout << "Present Gripper Load :" << dxl_present_load_1 << std::endl;
                            std::cout << "Present Goal Position :" << dxl_present_position_1 << std::endl;
                            _goal_pos_1 = -1;
                        }
                    }
                }

                if(_goal_pos_2 != -1){
                    if(dxl_step_2 == 1){
                        if((dxl_present_load_2 > 1200)){
                            std::cout << "Force Occured!" << std::endl;
                            std::cout << "Present Gripper Load :" << dxl_present_load_2 << std::endl;
                            std::cout << "Present Goal Position :" << dxl_present_position_2 << std::endl;
                            _goal_pos_2 = -1;
                        }
                    }
                    else{
                        if((dxl_present_load_2> _load_threshold) && (dxl_present_load_2 < 1000)){
                            std::cout << "Force Occured!" << std::endl;
                            std::cout << "Present Gripper Load :" << dxl_present_load_2 << std::endl;
                            std::cout << "Present Goal Position :" << dxl_present_position_2 << std::endl;
                            _goal_pos_2 = -1;
                        }
                    }
                }

            }

            if(((abs(_goal_pos_1 - dxl_present_position_1) < DXL_MOVING_STATUS_THRESHOLD))){
                _goal_pos_1 = -1;
            }
            if(((abs(_goal_pos_2 - dxl_present_position_2) < DXL_MOVING_STATUS_THRESHOLD))){
                _goal_pos_2 = -1;
            }

            if((_goal_pos_1 == -1) && (_goal_pos_2 == -1)){
                break;
            }


            if(_goal_pos_1 != -1){
                dxl_next_position_1 = dxl_prev_position_1 + dxl_step_1;
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_next_position_1, &dxl_error);
            }
            if(_goal_pos_2 != -1){
                dxl_next_position_2 = dxl_prev_position_2 + dxl_step_2;
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_next_position_2, &dxl_error);
            }

            if (dxl_comm_result != COMM_SUCCESS){
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                printf("Read present position : Failed to COMM_SUCCESS\n");
            }
            else if (dxl_error != 0){
                mp_gripper_packetHandler->printRxPacketError(dxl_error);
                printf("Read present position : dxl_error\n");
            }

            dxl_prev_position_1 = dxl_next_position_1;
            dxl_prev_position_2 = dxl_next_position_2;

        }while(true);

        dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position_1, &dxl_error);
        dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position_2, &dxl_error);

        gripper_status.present_pose_1 = dxl_present_position_1;
        gripper_status.present_pose_2 = dxl_present_position_2;
    }
    mtx_gripper_handle.unlock();

    emit SignalEditeGripperStatus(gripper_status);

    return true;
}

bool CGripper::GripperGoToThePositionLoadCheck_1(int _goal_pos_1, int _load_threshold){ // Go to The Position

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int dxl_step = 1;
    int dxl_next_position = 0;
    uint16_t dxl_prev_position = 0;
    uint16_t dxl_present_position = 0; // Present position
    uint16_t dxl_present_load = 0; // Present position

    mtx_gripper_handle.lock();
    {
        dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        dxl_prev_position = dxl_present_position;

        if((_goal_pos_1 - dxl_present_position) > 0)
            dxl_step = 1;
        else
            dxl_step = -1;

        do
        {
            // Read present position
            dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);

            if(_load_threshold != 0){// if 0, do not use force ctrl_load_threshold
                if(dxl_step == 1){
                    if((dxl_present_load > 1200)){
                        std::cout << "Force Occured!" << std::endl;
                        std::cout << "Present Gripper Load :" << dxl_present_load << std::endl;
                        std::cout << "Present Goal Position :" << dxl_present_position << std::endl;
                        mtx_gripper_handle.unlock();
                        return true;
                    }
                }
                else{
                    if((dxl_present_load > _load_threshold) && (dxl_present_load < 1000)){
                        std::cout << "Force Occured!" << std::endl;
                        std::cout << "Present Gripper Load :" << dxl_present_load << std::endl;
                        std::cout << "Present Goal Position :" << dxl_present_position << std::endl;
                        mtx_gripper_handle.unlock();
                        return true;
                    }
                }
            }

            dxl_next_position = dxl_prev_position + dxl_step;
            dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_next_position, &dxl_error);


            if (dxl_comm_result != COMM_SUCCESS){
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                printf("Read present position : Failed to COMM_SUCCESS\n");
            }
            else if (dxl_error != 0){
                mp_gripper_packetHandler->printRxPacketError(dxl_error);
                printf("Read present position : dxl_error\n");
            }

            dxl_prev_position = dxl_next_position;
        }while((abs(_goal_pos_1 - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }

    mtx_gripper_handle.unlock();

    return true;

}

bool CGripper::GripperGoToThePositionLoadCheck_2(int _goal_pos_2, int _load_threshold){ // Go to The Position

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int dxl_step = 1;
    int dxl_next_position = 0;
    uint16_t dxl_prev_position = 0;
    uint16_t dxl_present_position = 0; // Present position
    uint16_t dxl_present_load = 0; // Present position

    mtx_gripper_handle.lock();
    {
        dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        dxl_prev_position = dxl_present_position;

        if((_goal_pos_2 - dxl_present_position) > 0)
            dxl_step = 1;
        else
            dxl_step = -1;

        do
        {
            // Read present position
            dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);

            if(_load_threshold != 0){// if 0, do not use force ctrl_load_threshold
                if(dxl_step == 1){
                    if((dxl_present_load > 1200)){
                        std::cout << "Force Occured!" << std::endl;
                        std::cout << "Present Gripper Load :" << dxl_present_load << std::endl;
                        std::cout << "Present Goal Position :" << dxl_present_position << std::endl;
                        mtx_gripper_handle.unlock();
                        return true;
                    }
                }
                else{
                    if((dxl_present_load > _load_threshold) && (dxl_present_load < 1000)){
                        std::cout << "Force Occured!" << std::endl;
                        std::cout << "Present Gripper Load :" << dxl_present_load << std::endl;
                        std::cout << "Present Goal Position :" << dxl_present_position << std::endl;
                        mtx_gripper_handle.unlock();
                        return true;
                    }
                }
            }

            dxl_next_position = dxl_prev_position + dxl_step;
            dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_next_position, &dxl_error);


            if (dxl_comm_result != COMM_SUCCESS){
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                printf("Read present position : Failed to COMM_SUCCESS\n");
            }
            else if (dxl_error != 0){
                mp_gripper_packetHandler->printRxPacketError(dxl_error);
                printf("Read present position : dxl_error\n");
            }

            dxl_prev_position = dxl_next_position;
        }while((abs(_goal_pos_2 - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }

    mtx_gripper_handle.unlock();

    return true;

}












