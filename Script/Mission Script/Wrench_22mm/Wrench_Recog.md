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
VEHICLE_LOCALIZATION_ON_PANEL_STRUCT.desired_h_dst = 0.55
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

VEHICLE_LOCALIZATION_ON_PANEL_STRUCT.desired_h_dst = 0.55
VEHICLE_LOCALIZATION_ON_PANEL_FUNCTION()


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

## Step11: Kinova Arm Go Left

KINOVA_MANIPULATE_STRUCT.mode = 2

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ++0.03
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = 2.1374
KINOVA_MANIPULATE_STRUCT.pitch = 1.6015
KINOVA_MANIPULATE_STRUCT.yaw = -2.2821

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)


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

## Step6: Vehicle Wrench Recognition

DRIVE_BY_WRENCH_RECOG_STRUCT.desired_wrench_index = gi_wrench_hanger_index
DRIVE_BY_WRENCH_RECOG_STRUCT.lrf_v_dst = 1

DRIVE_BY_WRENCH_RECOG_FUNCTION()

A_Sleep(3000)

## Step6: Kinova Wrench Recognition

KINOVA_BY_WRENCH_RECOG_STRUCT.desired_wrench_index = gi_wrench_hanger_index

KINOVA_BY_WRENCH_RECOG_FUNCTION()

A_Sleep(3000)

##########################################_MISSION_END_##########################################
