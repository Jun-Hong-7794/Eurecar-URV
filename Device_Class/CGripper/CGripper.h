#ifndef CGRIPPER_H
#define CGRIPPER_H

#include <QtWidgets>
#include <QMutex>
#include <QThread>
#include <QVector>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>

#include <iostream>

#include "dynamixel_sdk.h"

#define DMX_PI                          3.1415926279
#define ADDR_MX_TORQUE_ENABLE           24
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_GOAL_SPEED              32
#define ADDR_MX_PRESENT_LOAD            40

#define PROTOCOL_MX_VERSION             1.0
#define PROTOCOL_PR_VERSION             2.0

#define DXL_ID                          1
#define BAUDRATE                        1000000
#define DEVICENAME_MX                      "/dev/ttyUSB0"
#define DEVICENAME_PR                      "/dev/ttyUSB1"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

//#define DXL_MINIMUM_POSITION_VALUE      3074//279
//#define DXL_MAXIMUM_POSITION_VALUE      2150//189

#define DXL_MOVING_STATUS_THRESHOLD     5

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
#define DXL3_ID                         3
#define DXL4_ID                         4

#define DXL_MINIMUM_POSITION_VALUE      100
#define DXL_MAXIMUM_POSITION_VALUE      4000
#define DXL_GOAL_VELOCITY_VALUE         300//1100
#define DXL_INITIAL_POSITION_VALUE      2300 //2300

#define DXL2_OFFSET                     300
#define ESC_ASCII_VALUE                 0x1b

/*
 *
 * Dynamixel Pro Protocal
 *
 */
#define DXL_PRO_STEP_PER_DEGREE         1419.23   //1 degree is 1419.23 steps
#define DXL_PRO_DEGREE_PER_T_STEP       0.71724   //0.71724 * (10^(-4))deg

#define ADDR_PR_TORQUE_ENABLE           562

#define ADDR_PR_IS_MOVING               610

#define ADDR_PR_GOAL_POSITION           596
#define ADDR_PR_PRESENT_POSITION        611
#define ADDR_PR_GOAL_SPEED              600

#define DXL_PR_GOAL_SPEED               2300
#define DXL_PR_INITIAL_POSITION_VALUE   0

#define LEN_PR_GOAL_POSITION            4
#define LEN_PR_PRESENT_POSITION         4
#define LEN_PR_PRESENT_SPEED            4
#define LEN_PR_IS_MOVING                1

#define DXL_PR_3_INIT_POSE              -125475
#define DXL_PR_4_INIT_POSE              125475

typedef struct _Gripper_Status{

    int present_pose_1;
    int present_pose_2;

    int present_load_1;
    int present_load_2;

}GRIPPER_STATUS;

typedef struct _Gripper_Size_Detection_Data{

    double x;//Trial Number
    double y;//Diff Angle Between two Dynamixel

}GRIPPER_DATA;


typedef struct _Sine_Equation_PARAMETER{

    /*
     * Sin Equation
     * y = a * sin(bx + c) + d
     */
    double a;//scale
    double b;//period

    double c;//x offset
    double d;//y offset

    int num_inlier;

}SINE_EQ_PARAM;

class CGripper: public QThread{
    Q_OBJECT

public:
    CGripper();
    ~CGripper();

private:
    bool fl_port_status;
    dynamixel::PortHandler *mp_portHandler;
    dynamixel::PacketHandler *mp_packetHandler;
    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler

    // Rotator
    dynamixel::PortHandler *mp_rotator_portHandler;
    dynamixel::PacketHandler *mp_rotator_packetHandler;

    dynamixel::GroupBulkRead *mp_pro_groupBulkRead_pos;
    dynamixel::GroupBulkRead *mp_pro_groupBulkRead_tor;

    //Gripper
    dynamixel::PortHandler *mp_gripper_portHandler;
    dynamixel::PacketHandler *mp_gripper_packetHandler;

    dynamixel::GroupBulkRead *mp_mx_groupBulkRead_pos;
    dynamixel::GroupBulkRead *mp_mx_groupBulkRead_tor;


    bool fl_init_dynamixel;
    bool fl_torque;

    bool fl_init_gripper;
    bool fl_torque_gripper;

    bool fl_init_rotator;
    bool fl_torque_rotator;

    bool dxl_comm_result;
    bool dxl_addparam_result;
    bool dxl_getdata_result;
    int dxl_goal_position;

    int m_dxl_pro_goal_position;
    int m_dxl_pro_current_position;

    int m_dxl_pro_catesian_goal_position;
    int m_dxl_pro_catesian_current_position;

    uint8_t dxl_error;
    uint16_t m_dxl1_present_position;
    uint16_t m_dxl2_present_position;
    int dxl3_present_position;
    int dxl4_present_position;
    uint16_t dxl1_present_load;
    uint16_t dxl2_present_load;

    GRIPPER_STATUS mstruct_gripper_status;
private://Mutex
    QMutex mtx_dmx_handle;
    QMutex mtx_gripper_handle;
    QMutex mtx_rotator_handle;
    QMutex mtx_gripper_status;

    //catesian pos
    double m_kinova_body_x;
    double m_kinova_body_y;

public:
    bool IsDmxInit();
    bool IsDmxTorqueOn();

    bool IsGripperInit();
    bool IsGripperTorqueOn();

    bool IsRotatorInit();
    bool IsRotatorTorqueOn();

    bool IsPortOpend();
public:
    SINE_EQ_PARAM EstimateSineEquation(std::vector<GRIPPER_DATA>& _gripper_data_vec);

public:
    bool InitDynamixel(char* _device_port = (char *)"/dev/ttyUSB0");
    void CloseDynamixel();

    bool DynamixelTorque(bool _onoff);
    bool DynamixelGoToThePosition(int _degree); // Go to The Position
    bool DynamixelGoToRelPosition(double _degree); // Go to Relative Position From Present Position

    bool DynamixelGoToThePositionUsingLoad(int _degree, int _load_threshold, int _load_threshold_dw = 800); // Go to The Position

    uint16_t DynamixelPresentPosition();
    uint16_t DynamixelPresentLoad();

    //Port
    bool GripperPortInit(char* _device_port = (char *)"/dev/ttyUSB0");
    bool RotatorPortInit(char* _device_port = (char *)"/dev/ttyUSB1");

    bool CloseDynamixelPort();

    //Rotator /*Dynamixel Pro*/
    bool InitRotator(char* _device_port = (char *)"/dev/ttyUSB1");
    void CloseRotator();
    bool RotatorTorque(bool _onoff);

    bool RotatorGoToThePosition(int _step); // Go to The Position
    bool RotatorGoToRelPosition(int _step); // Go to Relative Position From Present Position

    bool CatesianRotatorGoToThePosition(double _dist_error); // Go to The Position // _dist_error = current depth - ref depth [mm]
    bool CatesianRotatorGoToRelPosition(int _step); // Go to Relative Position From Present Position

    bool CatesianRotatorGoToOposite(bool _home);

    bool GetKinovaBodyPose(double &_x, double &_y);

    //Gripper
    bool InitGripper(char* _device_port = (char *)"/dev/ttyUSB0");
    void CloseGripper();

    bool GripperTorque(bool _onoff);
    bool GripperGoToThePosition(int _degree); // Go to The Position
    bool GripperGoToRelPosition(int _rel_pose_1, int _rel_pose_2); // Go to Relative Position From Present Position

    bool GripperGoToThePositionLoadCheck(int _goal_pos_1, int _goal_pos_2, int _load_threshold); // Go to The Position

    GRIPPER_STATUS GetGripperStatus();
    void SetGripperStatus(GRIPPER_STATUS);

signals:
    void SignalEditeGripperStatus(GRIPPER_STATUS _gripper);

};
#endif // CGRIPPER_H

























