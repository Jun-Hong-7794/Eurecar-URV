/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2

global_int gi_valve_size = 19
global_double gb_valve_rotation = 0

global_int gi_wrench_hanger_index = 2
/*gi_valve_size

# Title: Wrench Recognition

# Title: Valve Recognition

global_int gi_valve_size = 19
global_double gb_valve_rotation = 0

global_double gd_check_v_dst = 30
global_bool gb_check_v_dst_rst = false

## Step1: Vehicle Moving
VEHICLE_LOCALIZATION_ON_PANEL_STRUCT.desired_h_dst = 0.8
VEHICLE_LOCALIZATION_ON_PANEL_FUNCTION()

## Step0: Rotator
ROTATOR_STRUCT.desired_position = -195475
ROTATOR_STRUCT.fl_rotator_torque = true

ROTATOR_FUNCTION()
A_Sleep(500)

## Step1: Align to Panel
KINOVA_ALIGN_TO_PANEL.do_init_motion = true
KINOVA_ALIGN_TO_PANEL_FUNCTION()

A_Sleep(1000)

## Step2: Rotator
/*ROTATOR_STRUCT.desired_position = 0
ROTATOR_STRUCT.desired_position = -125475
ROTATOR_STRUCT.fl_rotator_torque = true

ROTATOR_FUNCTION()
A_Sleep(4000)

## Step0: Align to Panel
KINOVA_ALIGN_TO_PANEL.do_init_motion = false
KINOVA_ALIGN_TO_PANEL_FUNCTION()

A_Sleep(1000)


## Step5: Gripper Release

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 2800
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 2800
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = -2

GRIPPER_FORCE_CTRL_FUNCTION()

A_Sleep(500)

## Step6: KINOVA Angle Control(New, Using Dynamixel Pro-LRF)
/* mode =2 => Left
LRF_K_ANGLE_CTRL_STRUCT.mode = 2

LRF_K_ANGLE_CTRL_STRUCT.error = 0.3
LRF_K_ANGLE_CTRL_STRUCT.desired_angle = 0
/*LRF_K_ANGLE_CTRL_STRUCT.desired_angle = 0

LRF_K_ANGLE_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_K_ANGLE_CTRL_STRUCT.e_deg = 170

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 1

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()

A_Sleep(300)

## Step1: Vehicle Moving
VEHICLE_LOCALIZATION_ON_PANEL_STRUCT.desired_h_dst = 0.8
VEHICLE_LOCALIZATION_ON_PANEL_FUNCTION()

## Step3: LRF-Kinova Vertical CTRL(NEW Using Scara)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 4
LRF_K_VERTICAL_CTRL_STRUCT.force_option = false
LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 300
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(3000)


## Step4: KINOVA Angle Control(New, Using Dynamixel Pro-LRF)
/* mode =2 => Left
LRF_K_ANGLE_CTRL_STRUCT.mode = 3

LRF_K_ANGLE_CTRL_STRUCT.error = 0.3
LRF_K_ANGLE_CTRL_STRUCT.desired_angle = 0

LRF_K_ANGLE_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_K_ANGLE_CTRL_STRUCT.e_deg = 170

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 0.8

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()


## Step2: Vehicle Horizen Control(New, Using Kinova-LRF)

LRF_V_HORIZEN_CTRL_STRUCT.mode = 3

LRF_V_HORIZEN_CTRL_STRUCT.desired_h_dst = 850

LRF_V_HORIZEN_CTRL_STRUCT.error = 50

LRF_V_HORIZEN_CTRL_STRUCT.s_deg = 10
LRF_V_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_V_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_V_HORIZEN_CTRL_STRUCT.velocity = 58

LRF_V_HORIZEN_CTRL_STRUCT.loop_sleep = 30

LRF_V_HORIZEN_CTRL_FUNCTION()

A_Sleep(500)


## Step4: KINOVA Angle Control(New, Using Dynamixel Pro-LRF)
/* mode =2 => Left
LRF_K_ANGLE_CTRL_STRUCT.mode = 3

LRF_K_ANGLE_CTRL_STRUCT.error = 0.3
LRF_K_ANGLE_CTRL_STRUCT.desired_angle = 0

LRF_K_ANGLE_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_K_ANGLE_CTRL_STRUCT.e_deg = 170

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 0.3

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()


## Step3: LRF-Kinova Vertical CTRL(NEW)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 3
LRF_K_VERTICAL_CTRL_STRUCT.force_option = false
LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 300
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(500)

## Step3: LRF-Kinova Horizen CTRL(NEW)

LRF_K_HORIZEN_CTRL_STRUCT.mode = 3

LRF_K_HORIZEN_CTRL_STRUCT.only_sensing_moving = false

LRF_K_HORIZEN_CTRL_STRUCT.desired_h_dst = 850

LRF_K_HORIZEN_CTRL_STRUCT.error = 3

LRF_K_HORIZEN_CTRL_STRUCT.s_deg = 10 
LRF_K_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_K_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_HORIZEN_CTRL_STRUCT.loop_sleep = 30 

LRF_K_HORIZEN_CTRL_FUNCTION()

A_Sleep(500)

## Step5: Kinova Arm Dw

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = 0.2238

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1500)

## Step11: Kinova Arm Go Back

KINOVA_MANIPULATE_STRUCT.mode = 2

KINOVA_MANIPULATE_STRUCT.x = --0.05
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = 2.1374
KINOVA_MANIPULATE_STRUCT.pitch = 1.6015
KINOVA_MANIPULATE_STRUCT.yaw = -2.2821

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)
## Step6: Wrench Recognition

B_Sleep(2000)

WRENCH_RECOGNITION_STRUCT.num_of_wrench = 6
WRENCH_RECOGNITION_STRUCT.loop_count = 50
WRENCH_RECOGNITION_STRUCT.valve_size = gi_valve_size
gi_wrench_hanger_index = WRENCH_RECOGNITION_FUNCTION()
/*WRENCH_RECOGNITION_STRUCT.valve_size = 16
/*WRENCH_RECOGNITION_FUNCTION()

A_Sleep(1000)

## Step13: Align to Panel
IF(gi_wrench_hanger_index != -1)
KINOVA_ALIGN_TO_PANEL.do_init_motion = false
KINOVA_ALIGN_TO_PANEL_FUNCTION()

A_Sleep(1000)

ELSE(GoTo: 0)

##########################################_MISSION_END_##########################################
