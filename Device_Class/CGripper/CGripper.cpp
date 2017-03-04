#include "CGripper.h"

CGripper::CGripper()
{
    fl_init_dynamixel = false;
    fl_torque = false;

    fl_port_status = false;
    fl_torque_rotator = false;
    fl_torque_gripper = false;

    fl_init_rotator = false;

    m_dxl1_present_position = 0;
    m_dxl2_present_position = 0;
}

CGripper::~CGripper()
{
    CloseDynamixel();

    if(IsRotatorInit())
        CloseRotator();
    else if(IsGripperInit())
        CloseGripper();
}


bool CGripper::InitDynamixel(char* _device_port){

    if(fl_init_dynamixel)
        return true;

    mtx_dmx_handle.lock();
    {
        mp_portHandler = dynamixel::PortHandler::getPortHandler(_device_port);
        mp_packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_MX_VERSION);

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
    fl_port_status = false;
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

bool CGripper::IsPortOpend(){
    return fl_port_status;
}

bool CGripper::IsGripperInit(){
    if(!fl_init_gripper) return false;
    return true;
}


bool CGripper::IsGripperTorqueOn(){
    if(!fl_torque_gripper) return false;
    return true;
}

bool CGripper::IsRotatorInit(){

    return fl_init_rotator;
}

bool CGripper::IsRotatorTorqueOn(){

    return fl_torque_rotator;
}


bool CGripper::RotatorPortInit(char* _device_port){

    mp_rotator_portHandler = dynamixel::PortHandler::getPortHandler(_device_port);
    mp_rotator_packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_PR_VERSION);

    mp_pro_groupBulkRead_pos = new dynamixel::GroupBulkRead(mp_rotator_portHandler, mp_rotator_packetHandler);
    mp_pro_groupBulkRead_tor = new dynamixel::GroupBulkRead(mp_rotator_portHandler, mp_rotator_packetHandler);

    // Open port
    if (mp_rotator_portHandler->openPort()){
        printf("Succeeded to open the port!\n");
    }
    else{
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getchar();
        return false;
    }

    // Set port baudrate
    if (mp_rotator_portHandler->setBaudRate(BAUDRATE)){
        printf("Succeeded to change the baudrate!\n");
    }
    else{
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getchar();
        return false;
    }

    fl_port_status = true;

    return true;
}

bool CGripper::GripperPortInit(char* _device_port){

    mp_gripper_portHandler = dynamixel::PortHandler::getPortHandler(_device_port);
    mp_gripper_packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_MX_VERSION);

    mp_mx_groupBulkRead_pos = new dynamixel::GroupBulkRead(mp_gripper_portHandler, mp_gripper_packetHandler);
    mp_mx_groupBulkRead_tor = new dynamixel::GroupBulkRead(mp_gripper_portHandler, mp_gripper_packetHandler);

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

    fl_port_status = true;

    return true;
}

//------------------------------------------------
//
// Rotator /*Dynamixel Pro*/
//
//------------------------------------------------

bool CGripper::InitRotator(char* _device_port){

    if(fl_init_rotator)
        return true;

    if(!RotatorPortInit(_device_port))
        return false;

    dxl_goal_position = DXL_PR_INITIAL_POSITION_VALUE;


    //ID - 3 Torque ON /*Dynamixel Pro*/
    RotatorTorque(true);

    //ID - 3 Present Poisition Setting
    dxl_addparam_result = mp_pro_groupBulkRead_pos->addParam(DXL3_ID, ADDR_PR_PRESENT_POSITION, LEN_PR_PRESENT_POSITION);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL3 present position addparam failed\n", DXL3_ID);
        return false;
    }

    //ID - 3 Goal Speed Setting
    dxl_comm_result = mp_rotator_packetHandler->write4ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_GOAL_SPEED, DXL_PR_GOAL_SPEED, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);

    else if(dxl_error != 0)
        mp_rotator_packetHandler->printRxPacketError(dxl_error);

    //ID - 3 Operating Mode Setting
    dxl_comm_result = mp_rotator_packetHandler->write4ByteTxRx(mp_rotator_portHandler, DXL3_ID, 11, 3/*+DXL1_OFFSET*/, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_rotator_packetHandler->printRxPacketError(dxl_error);

    //ID - 3 Goal Positoin Setting
    dxl_comm_result = mp_rotator_packetHandler->write4ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_GOAL_POSITION, dxl_goal_position - 5000/*+DXL1_OFFSET*/, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_rotator_packetHandler->printRxPacketError(dxl_error);

    dxl_comm_result = mp_pro_groupBulkRead_pos->txRxPacket();

    msleep(1500);

    dxl_comm_result = mp_rotator_packetHandler->write4ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_GOAL_POSITION, dxl_goal_position/*+DXL1_OFFSET*/, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_rotator_packetHandler->printRxPacketError(dxl_error);

    dxl_comm_result = mp_pro_groupBulkRead_pos->txRxPacket();

    //ID - 3 Connection Check
    if(dxl_comm_result != COMM_SUCCESS)
        mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);

    dxl_getdata_result = mp_pro_groupBulkRead_pos->isAvailable(DXL3_ID, ADDR_PR_PRESENT_POSITION, LEN_PR_PRESENT_POSITION);

    if(!dxl_getdata_result)
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL3 present position getdata failed", DXL3_ID);

    //ID - 3 Read Position
    uint32_t present_pose_3 = 0;
    mp_rotator_packetHandler->read4ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_PRESENT_POSITION, &present_pose_3, &dxl_error);
    dxl3_present_position = mp_pro_groupBulkRead_pos->getData(DXL3_ID, ADDR_PR_PRESENT_POSITION, LEN_PR_PRESENT_POSITION);

    fl_init_rotator = true;

    m_dxl_pro_current_position = 0;

    return true;
}

void CGripper::CloseRotator(){

    RotatorTorque(false);

    mp_rotator_portHandler->closePort();

    fl_torque_rotator = false;
}

bool CGripper::RotatorTorque(bool _onoff){

    if(_onoff){
        if(fl_torque_rotator) return false;

        //ID - 3 Torque Setting
        dxl_comm_result = mp_rotator_packetHandler->write1ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS)
            mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_rotator_packetHandler->printTxRxResult(dxl_error);
        else
            printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);

        fl_torque_rotator = true;
    }
    else{
        if(!fl_torque_rotator) return false;

        //ID - 3 Torque Setting
        dxl_comm_result = mp_rotator_packetHandler->write1ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS)
            mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_rotator_packetHandler->printTxRxResult(dxl_error);
        else
            printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);

        fl_torque_rotator = false;
    }

    return true;
}

bool CGripper::RotatorGoToThePosition(int _step){ // Go to The Position

    if(!fl_init_rotator)
        return false;

    mtx_gripper_handle.lock();
    m_dxl_pro_current_position = _step;

    m_dxl_pro_goal_position = m_dxl_pro_current_position;
    dxl_comm_result = mp_rotator_packetHandler->write4ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_GOAL_POSITION, m_dxl_pro_goal_position/*+DXL1_OFFSET*/, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);

    else if(dxl_error != 0)
        mp_rotator_packetHandler->printRxPacketError(dxl_error);

//    do{
//        mp_rotator_packetHandler->read1ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_IS_MOVING, &is_moving, &dxl_error);

//    }while(is_moving > 0);


    mtx_gripper_handle.unlock();
    return true;
}

bool CGripper::RotatorGoToRelPosition(int _step){ // Go to Relative Position From Present Position

    if(!fl_init_rotator)
        return false;

    mtx_gripper_handle.lock();
//    dxl3_present_position = mp_pro_groupBulkRead_pos->getData(DXL3_ID, ADDR_PR_PRESENT_POSITION, LEN_PR_PRESENT_POSITION);

//    dxl3_present_position = dxl3_present_position + _step;

    m_dxl_pro_current_position += _step;
    m_dxl_pro_goal_position = m_dxl_pro_current_position;

    dxl_comm_result = mp_rotator_packetHandler->write4ByteTxRx(mp_rotator_portHandler, DXL3_ID, ADDR_PR_GOAL_POSITION, m_dxl_pro_goal_position, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
        mp_rotator_packetHandler->printTxRxResult(dxl_comm_result);

    else if(dxl_error != 0)
        mp_rotator_packetHandler->printRxPacketError(dxl_error);
    mtx_gripper_handle.unlock();
    return true;
}

//------------------------------------------------
//
// Gripper /*Dynamixel MX28 & 64*/
//
//------------------------------------------------

bool CGripper::InitGripper(char* _device_port){

    fl_torque_gripper = false;

    if(!GripperPortInit(_device_port))
        return false;

    //ID - 1,2 Torque ON
    GripperTorque(true);

    dxl_goal_position = DXL_INITIAL_POSITION_VALUE;

    //ID - 1 Present Poisition Setting
    dxl_addparam_result = mp_mx_groupBulkRead_pos->addParam(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position addparam failed\n", DXL1_ID);
        return false;
    }
    //ID - 2 Present Poisition Setting
    dxl_addparam_result = mp_mx_groupBulkRead_pos->addParam(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position addparam failed\n", DXL2_ID);
        return false;
    }

    //ID - 1 Present Load Setting
    dxl_addparam_result = mp_mx_groupBulkRead_tor->addParam(DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present load addparam failed\n", DXL1_ID);
        return false;
    }
    //ID - 2 Present Load Setting
    dxl_addparam_result = mp_mx_groupBulkRead_tor->addParam(DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);
    if(dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present load addparam failed\n", DXL2_ID);
        return false;
    }

    //ID - 1 Goal Speed Setting
    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_SPEED, DXL_GOAL_VELOCITY_VALUE, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

    else if(dxl_error != 0) mp_gripper_packetHandler->printRxPacketError(dxl_error);

    //ID - 2 Goal Speed Setting
    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_SPEED, DXL_GOAL_VELOCITY_VALUE, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

    else if(dxl_error != 0) mp_gripper_packetHandler->printRxPacketError(dxl_error);

    //ID - 1 Goal Positoin Setting
    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position + 200/*+DXL1_OFFSET*/, &dxl_error);
    m_dxl1_present_position = DXL_INITIAL_POSITION_VALUE;

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_gripper_packetHandler->printRxPacketError(dxl_error);

    msleep(500);

    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position/*+DXL1_OFFSET*/, &dxl_error);
    m_dxl1_present_position = DXL_INITIAL_POSITION_VALUE;

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_gripper_packetHandler->printRxPacketError(dxl_error);

    //ID - 2 Goal Positoin Setting
    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position + DXL2_OFFSET + 200, &dxl_error);
    m_dxl2_present_position = DXL_INITIAL_POSITION_VALUE;

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_gripper_packetHandler->printRxPacketError(dxl_error);

    msleep(500);

    dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position + DXL2_OFFSET, &dxl_error);
    m_dxl2_present_position = DXL_INITIAL_POSITION_VALUE;

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        mp_gripper_packetHandler->printRxPacketError(dxl_error);


    fl_init_gripper = true;

    dxl_comm_result = mp_mx_groupBulkRead_pos->txRxPacket();

    //ID - 1 Connection Check
    if(dxl_comm_result != COMM_SUCCESS) mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
    dxl_getdata_result = mp_mx_groupBulkRead_pos->isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    if(!dxl_getdata_result) fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position getdata failed", DXL1_ID);

    //ID - 2 Connection Check
    if(!dxl_getdata_result) fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position getdata failed", DXL1_ID);
    dxl_getdata_result = mp_mx_groupBulkRead_pos->isAvailable(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    if(!dxl_getdata_result) fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position getdata failed", DXL2_ID);

    return true;
}

void CGripper::CloseGripper(){

    GripperTorque(false);

    mp_gripper_portHandler->closePort();

    fl_init_gripper= false;
    fl_port_status = false;
}

bool CGripper::GripperTorque(bool _onoff){

    if(_onoff){
        if(fl_torque_gripper) return false;

        //ID - 1 Torque Setting
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printTxRxResult(dxl_error);
        else
            printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);

        //ID - 2 Torque Setting
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printTxRxResult(dxl_error);
        else
            printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);

        fl_torque_gripper = true;
    }
    else{
        if(!fl_torque_gripper) return false;

        //ID - 1 Torque Setting
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printTxRxResult(dxl_error);

        //ID - 2 Torque Setting
        dxl_comm_result = mp_gripper_packetHandler->write1ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printTxRxResult(dxl_error);

        fl_torque_gripper = false;
    }

    return true;
}

bool CGripper::GripperGoToThePosition(int _degree){ // Go to The Position

    dxl_comm_result = mp_mx_groupBulkRead_pos->txRxPacket();

    if(dxl_comm_result != COMM_SUCCESS)
        mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

    dxl_getdata_result = mp_mx_groupBulkRead_pos->isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    if(!dxl_getdata_result)
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position getdata failed", DXL1_ID);

    dxl_getdata_result = mp_mx_groupBulkRead_pos->isAvailable(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    if(!dxl_getdata_result)
        fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position getdata failed", DXL2_ID);

    int i = 0;
//    int previous = 0;
    int cnt = 0;

    if(_degree >= (m_dxl1_present_position + m_dxl2_present_position)/2){// When Release Gripper....

        m_dxl1_present_position = _degree;
        dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, m_dxl1_present_position/*+DXL1_OFFSET*/, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printRxPacketError(dxl_error);

        m_dxl2_present_position = _degree;
        dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, m_dxl2_present_position + DXL2_OFFSET, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS)
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0)
            mp_gripper_packetHandler->printRxPacketError(dxl_error);
    }
    else{// When Grasp....
        while(true){//To use force feed back, move dynamixel step by step

//            previous = (dxl1_present_position + dxl2_present_position)/2;
            sleep(0.5);
            if(m_dxl2_present_position > _degree){
                i++;
                m_dxl2_present_position = (m_dxl1_present_position + m_dxl2_present_position)/2 - 5*i;
            }

            dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, m_dxl1_present_position/*+DXL1_OFFSET*/, &dxl_error);

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            else if(dxl_error != 0)
                mp_gripper_packetHandler->printRxPacketError(dxl_error);

            dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, m_dxl2_present_position + DXL2_OFFSET, &dxl_error);

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            else if(dxl_error != 0)
                mp_gripper_packetHandler->printRxPacketError(dxl_error);

            dxl_comm_result = mp_mx_groupBulkRead_pos->txRxPacket();

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            dxl_getdata_result = mp_mx_groupBulkRead_pos->isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present position getdata failed", DXL1_ID);

            dxl_getdata_result = mp_mx_groupBulkRead_pos->isAvailable(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present position getdata failed", DXL2_ID);

            dxl_comm_result = mp_mx_groupBulkRead_tor->txRxPacket();

            if(dxl_comm_result != COMM_SUCCESS)
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);

            dxl_getdata_result = mp_mx_groupBulkRead_tor->isAvailable(DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL1 present load getdata failed", DXL1_ID);

            dxl_getdata_result = mp_mx_groupBulkRead_tor->isAvailable(DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);

            if(!dxl_getdata_result)
                fprintf(stderr, "[ID:%03d] groupBulkRead DXL2 present load getdata failed", DXL2_ID);

            dxl1_present_load= mp_mx_groupBulkRead_tor->getData(DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);
            dxl2_present_load= mp_mx_groupBulkRead_tor->getData(DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);

            if(((dxl1_present_load > 180)&&(dxl1_present_load < 1023))||((dxl2_present_load > 300)&&(dxl2_present_load < 1023)))
                break;

            if((m_dxl1_present_position + m_dxl2_present_position)/2 - _degree <= 10) break;

            else if(((m_dxl1_present_position + m_dxl2_present_position)/2 - _degree > 10)&&((m_dxl1_present_position + m_dxl2_present_position)/2 - _degree <= 25)){
                cnt++;
                if(cnt > 100) break;
            }
        }
    }
    return true;
}

bool CGripper::GripperGoToRelPosition(int _rel_pose_1, int _rel_pose_2){ // Go to Relative Position From Present Position

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result


    GRIPPER_STATUS gripper_status;

    mtx_gripper_handle.lock();
    {

        m_dxl1_present_position = m_dxl1_present_position + _rel_pose_1;
        dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, m_dxl1_present_position , &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS){
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
            mtx_gripper_handle.unlock();
            return false;
        }

        m_dxl2_present_position = m_dxl2_present_position + _rel_pose_2;
        dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, m_dxl2_present_position + DXL2_OFFSET, &dxl_error);

        if(dxl_comm_result != COMM_SUCCESS){
            mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
            mtx_gripper_handle.unlock();
            return false;
        }

        gripper_status.present_pose_1 = m_dxl1_present_position;
        gripper_status.present_pose_2 = m_dxl2_present_position;
    }
    mtx_gripper_handle.unlock();

    emit SignalEditeGripperStatus(gripper_status);

    return true;
}

bool CGripper::GripperGoToThePositionLoadCheck(int _goal_pos_1, int _goal_pos_2, int _load_threshold){ // Go to The Position

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int dxl_step_1 = 1;
    uint16_t dxl_present_load_1 = 0; // Present position

    int dxl_step_2 = 1;
    uint16_t dxl_present_load_2 = 0; // Present position

    GRIPPER_STATUS gripper_status;

    mtx_gripper_handle.lock();
    {

        if(_load_threshold == -2){// Go to the position no load and no unit step

            if(_goal_pos_1 > 0){
                m_dxl1_present_position = _goal_pos_1;
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, m_dxl1_present_position, &dxl_error);

                if(dxl_comm_result != COMM_SUCCESS){
                    mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                    mtx_gripper_handle.unlock();
                    return false;
                }
                else if(dxl_error != 0)
                    mp_gripper_packetHandler->printRxPacketError(dxl_error);
            }
            if(_goal_pos_2 > 0){
                m_dxl2_present_position = _goal_pos_2;
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, m_dxl2_present_position + DXL2_OFFSET, &dxl_error);

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

        if((_goal_pos_1 - m_dxl1_present_position) > 0)
            dxl_step_1 = 1;
        else
            dxl_step_1 = -1;

        if((_goal_pos_2 - m_dxl2_present_position) > 0)
            dxl_step_2 = 1;
        else
            dxl_step_2 = -1;

        do
        {
            if(_goal_pos_1 != -1){
                // Read present position
                dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load_1, &dxl_error);
            }
            if(dxl_comm_result != COMM_SUCCESS){
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                msleep(50);
//                continue;
            }
            msleep(1);

            if(_goal_pos_2 != -1){
                // Read present position
                dxl_comm_result = mp_gripper_packetHandler->read2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load_2, &dxl_error);
            }
            if(dxl_comm_result != COMM_SUCCESS){
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                msleep(50);
//                continue;
            }
            msleep(1);

            gripper_status.present_pose_1 = m_dxl1_present_position;
            gripper_status.present_pose_2 = m_dxl2_present_position;

            gripper_status.present_load_1 = dxl_present_load_1;
            gripper_status.present_load_2 = dxl_present_load_2;

            SetGripperStatus(gripper_status);
            emit SignalEditeGripperStatus(gripper_status);

            if(_load_threshold != 0){// if 0, do not use force ctrl_load_threshold

                if(_goal_pos_1 != -1){
                    if(dxl_step_1 == 1){
                        if((dxl_present_load_1 > 1800)){
//                            std::cout << "Force Occured!" << std::endl;
//                            std::cout << "Present Gripper Load :" << dxl_present_load_1 << std::endl;
//                            std::cout << "Present Goal Position :" << m_dxl1_present_position << std::endl;
                            _goal_pos_1 = -1;
                        }
                    }
                    else{
                        if((dxl_present_load_1> _load_threshold) && (dxl_present_load_1 < 1000)){
//                            std::cout << "Force Occured!" << std::endl;
//                            std::cout << "Present Gripper Load :" << dxl_present_load_1 << std::endl;
//                            std::cout << "Present Goal Position :" << m_dxl1_present_position << std::endl;
                            _goal_pos_1 = -1;
                        }
                    }
                }

                if(_goal_pos_2 != -1){
                    if(dxl_step_2 == 1){
                        if((dxl_present_load_2 > 1800)){
//                            std::cout << "Force Occured!" << std::endl;
//                            std::cout << "Present Gripper Load :" << dxl_present_load_2 << std::endl;
//                            std::cout << "Present Goal Position :" << m_dxl2_present_position << std::endl;
                            _goal_pos_2 = -1;
                        }
                    }
                    else{
                        if((dxl_present_load_2> _load_threshold) && (dxl_present_load_2 < 1000)){
//                            std::cout << "Force Occured!" << std::endl;
//                            std::cout << "Present Gripper Load :" << dxl_present_load_2 << std::endl;
//                            std::cout << "Present Goal Position :" << m_dxl2_present_position << std::endl;
                            _goal_pos_2 = -1;
                        }
                    }
                }

            }

            if(((abs(_goal_pos_1 - m_dxl1_present_position) < DXL_MOVING_STATUS_THRESHOLD))){
                _goal_pos_1 = -1;
            }
            if(((abs(_goal_pos_2 - m_dxl2_present_position) < DXL_MOVING_STATUS_THRESHOLD))){
                _goal_pos_2 = -1;
            }

            if((_goal_pos_1 == -1) && (_goal_pos_2 == -1)){
                break;
            }


            if(_goal_pos_1 != -1){
                m_dxl1_present_position = m_dxl1_present_position + dxl_step_1;
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, m_dxl1_present_position, &dxl_error);
            }
            if(_goal_pos_2 != -1){
                m_dxl2_present_position = m_dxl2_present_position + dxl_step_2;
                dxl_comm_result = mp_gripper_packetHandler->write2ByteTxRx(mp_gripper_portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, m_dxl2_present_position + DXL2_OFFSET, &dxl_error);
            }

            if (dxl_comm_result != COMM_SUCCESS){
                mp_gripper_packetHandler->printTxRxResult(dxl_comm_result);
                printf("Read present position : Failed to COMM_SUCCESS\n");
            }
            else if (dxl_error != 0){
                mp_gripper_packetHandler->printRxPacketError(dxl_error);
                printf("Read present position : dxl_error\n");
            }

            usleep(500);

        }while(true);

        gripper_status.present_pose_1 = m_dxl1_present_position;
        gripper_status.present_pose_2 = m_dxl2_present_position;
    }
    mtx_gripper_handle.unlock();

    SetGripperStatus(gripper_status);
    emit SignalEditeGripperStatus(gripper_status);

    return true;
}

GRIPPER_STATUS CGripper::GetGripperStatus(){

    GRIPPER_STATUS gripper_status;

    mtx_gripper_status.lock();
    {
        gripper_status = mstruct_gripper_status;
    }
    mtx_gripper_status.unlock();

    return gripper_status;
}

void CGripper::SetGripperStatus(GRIPPER_STATUS _gripper_status){

    mtx_gripper_status.lock();
    {
        mstruct_gripper_status = _gripper_status;
    }
    mtx_gripper_status.unlock();
}

//----------------------------------------------------------------
//
//                            Calculate
//
//----------------------------------------------------------------

SINE_EQ_PARAM CGripper::EstimateSineEquation(std::vector<GRIPPER_DATA>& _gripper_data_vec){

    SINE_EQ_PARAM sine_equation_parameter;
    SINE_EQ_PARAM optimal_sine_equation_parameter;

    int iteration = 500;

    int random_num_1 = 0;
    int random_num_2 = 0;
    int random_num_3 = 0;
    int random_num_4 = 0;

    int random_max_num = _gripper_data_vec.size();

    if(random_max_num <= 0){

        sine_equation_parameter.a = 0;
        sine_equation_parameter.b = 0;
        sine_equation_parameter.c = 0;
        sine_equation_parameter.d = 0;

        return sine_equation_parameter;
    }

    std::vector<SINE_EQ_PARAM> sine_eq_vector;

    std::srand((unsigned int)time(NULL));

    //Get Line Equations candidate using random sampling
    for(int i = 0; i < iteration; i++){

        random_num_1 = std::rand() % random_max_num;
        random_num_2 = std::rand() % random_max_num;
        random_num_3 = std::rand() % random_max_num;
        random_num_4 = std::rand() % random_max_num;

        _gripper_data_vec.at(random_num_1);
        _gripper_data_vec.at(random_num_2);
        _gripper_data_vec.at(random_num_3);
        _gripper_data_vec.at(random_num_4);

        //Calculate Sine Function Parameter
        // y = a * sin(b * x + c) + d

//        sine_equation_parameter.a
//        sine_equation_parameter.b
//        sine_equation_parameter.c
//        sine_equation_parameter.d

        sine_equation_parameter.num_inlier = 0;

        sine_eq_vector.push_back(sine_equation_parameter);
    }

//    double sine_parm_a = 0.0;
//    double sine_parm_b = 0.0;
//    double sine_parm_c = 0.0;
//    double sine_parm_d = 0.0;

    double inlier_standard = 7;

    //Calculate Inlier Point Within inlier_standard
    for(unsigned int i = 0; i < sine_eq_vector.size(); i++){

//        sine_parm_a = sine_eq_vector.at(i).a;
//        sine_parm_b = sine_eq_vector.at(i).b;
//        sine_parm_c = sine_eq_vector.at(i).c;
//        sine_parm_d = sine_eq_vector.at(i).d;

        for(unsigned int j = 0; j < sine_eq_vector.size(); j++){

            double distance = 0;

//            line_son = fabs(line_parm_a*_point_vec.at(j).x - _point_vec.at(j).y + line_parm_b);
//            line_parent = sqrt(pow(line_parm_a,2.0) + pow(1.0,2.0));
//            line_distance = (line_son / line_parent);

            if(distance <= inlier_standard){
                sine_eq_vector.at(i).num_inlier++;
            }
        }
    }

    //Find Maximum inlier Sine Equation Parameter
    int sine_eq_max_count = 0;

    for(unsigned int i = 0; i < sine_eq_vector.size(); i++){

        if(sine_eq_max_count < sine_eq_vector.at(i).num_inlier){

            sine_eq_max_count = sine_eq_vector.at(i).num_inlier;
            optimal_sine_equation_parameter = sine_eq_vector.at(i);
        }
    }

    return optimal_sine_equation_parameter;
}










