#ifndef CGRIPPER_H
#define CGRIPPER_H

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>

#include <iostream>

#include "dynamixel_sdk.h"

#define ADDR_MX_TORQUE_ENABLE           24
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_GOAL_SPEED              32
#define ADDR_MX_PRESENT_LOAD            40

#define PROTOCOL_VERSION                1.0

#define DXL_ID                          1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

//#define DXL_MINIMUM_POSITION_VALUE      3074//279
//#define DXL_MAXIMUM_POSITION_VALUE      2150//189

#define DXL_MOVING_STATUS_THRESHOLD     10

#define DXL_STEP_PER_DEGREE             11.363636364 //1 degree is 11.3777778 steps
#define DXL_DEGREE_PER_STEP             0.088      //1 step is 0.088 steps

/*
 *
 * Gipper
 *
 */

#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_PRESENT_LOAD             2
#define LEN_MX_PRESENT_SPEED            2
#define DXL1_ID                         1
#define DXL2_ID                         2

#define DXL_MINIMUM_POSITION_VALUE      100
#define DXL_MAXIMUM_POSITION_VALUE      4000
#define DXL_GOAL_VELOCITY_VALUE         1100
#define DXL_INITIAL_POSITION_VALUE      2300 //2300
#define DXL1_OFFSET                     160


#define ESC_ASCII_VALUE                 0x1b


class CGripper{
public:
    CGripper();

private:
    dynamixel::PortHandler *mp_portHandler;

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *mp_packetHandler;

    //Gripper
    dynamixel::PortHandler *mp_gripper_portHandler;
    dynamixel::PacketHandler *mp_gripper_packetHandler;

    dynamixel::GroupBulkRead *mp_groupBulkRead_pos;
    dynamixel::GroupBulkRead *mp_groupBulkRead_tor;


    bool fl_init_dynamixel;
    bool fl_torque;

    bool fl_init_gripper;
    bool fl_torque_gripper;

    bool dxl_comm_result;
    bool dxl_addparam_result;
    bool dxl_getdata_result;
    int dxl_goal_position;

    uint8_t dxl_error;
    uint16_t dxl1_present_position;
    uint16_t dxl2_present_position;
    uint16_t dxl1_present_load;
    uint16_t dxl2_present_load;
public:
    bool IsDmxInit();
    bool IsDmxTorqueOn();

    bool IsGripperInit();
    bool IsGripperTorqueOn();

    uint16_t Dmx_Current_Pos();
public:
    bool InitDynamixel();
    void CloseDynamixel();

    bool DynamixelTorque(bool _onoff);
    bool DynamixelGoToThePosition(int _degree); // Go to The Position
    bool DynamixelGoToRelPosition(double _degree); // Go to Relative Position From Present Position

    //Gripper
    bool InitGripper();
    void CloseGripper();

    bool GripperTorque(bool _onoff);
    bool GripperGoToThePosition(int _degree); // Go to The Position
    bool GripperGoToRelPosition(int _degree); // Go to Relative Position From Present Position
};

#endif // CGRIPPER_H












