/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Valve Recognition

global_int gi_valve_size = 19
global_double gb_valve_rotation = 60

global_double gd_check_v_dst = 30
global_bool gb_check_v_dst_rst = false

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
A_Sleep(2500)

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

## Step3: LRF-Kinova Vertical CTRL(NEW Using Scara)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 4
LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 290
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(3000)

## Step3: LRF-Kinova Vertical CTRL(NEW)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 2
LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 240
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(300)

## Step4: Vehicle Horizen Control(New, Using Kinova-LRF)

LRF_V_HORIZEN_CTRL_STRUCT.mode = 2

LRF_V_HORIZEN_CTRL_STRUCT.desired_h_dst = 325

LRF_V_HORIZEN_CTRL_STRUCT.error = 20

LRF_V_HORIZEN_CTRL_STRUCT.s_deg = 10
LRF_V_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_V_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_V_HORIZEN_CTRL_STRUCT.velocity = 65

LRF_V_HORIZEN_CTRL_STRUCT.loop_sleep = 30

LRF_V_HORIZEN_CTRL_FUNCTION()

A_Sleep(300)

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

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 0.3

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()

A_Sleep(300)

## Step7: LRF-Kinova Vertical CTRL(NEW)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 2

LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 240
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(500)

## Step8: KINOVA Angle Control(New, Using Dynamixel Pro-LRF)
/* mode =2 => Left
LRF_K_ANGLE_CTRL_STRUCT.mode = 2

LRF_K_ANGLE_CTRL_STRUCT.error = 0.1
LRF_K_ANGLE_CTRL_STRUCT.desired_angle = 0
/*LRF_K_ANGLE_CTRL_STRUCT.desired_angle = 0

LRF_K_ANGLE_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_K_ANGLE_CTRL_STRUCT.e_deg = 170

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 0.2

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()

A_Sleep(300)

## Step10: LRF-Kinova Horizen CTRL(NEW)

LRF_K_HORIZEN_CTRL_STRUCT.mode = 2

LRF_K_HORIZEN_CTRL_STRUCT.only_sensing_moving = false

LRF_K_HORIZEN_CTRL_STRUCT.desired_h_dst = 345
/*Center Point
/*LRF_K_HORIZEN_CTRL_STRUCT.desired_h_dst = 335

LRF_K_HORIZEN_CTRL_STRUCT.error = 2.0

LRF_K_HORIZEN_CTRL_STRUCT.s_deg = 10 
LRF_K_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_K_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 800

LRF_K_HORIZEN_CTRL_STRUCT.loop_sleep = 10 

LRF_K_HORIZEN_CTRL_FUNCTION()

A_Sleep(500)


## Step11: Kinova Arm Down

KINOVA_MANIPULATE_STRUCT.mode = 2

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ==
/*KINOVA_MANIPULATE_STRUCT.z = 0.1583
KINOVA_MANIPULATE_STRUCT.z = 0.1723

KINOVA_MANIPULATE_STRUCT.roll = 2.1374
KINOVA_MANIPULATE_STRUCT.pitch = 1.5575
KINOVA_MANIPULATE_STRUCT.yaw = -2.2821

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)

## Step12: Kinova Force Control(Go Forward)

KINOVA_FORCE_CTRL_STRUCT.step_count = 100

KINOVA_FORCE_CTRL_STRUCT.mode = 1

KINOVA_FORCE_CTRL_STRUCT.force_threshold = 0
KINOVA_FORCE_CTRL_STRUCT.force_threshold_x = 12
KINOVA_FORCE_CTRL_STRUCT.force_threshold_y = 0
KINOVA_FORCE_CTRL_STRUCT.force_threshold_z = 0

KINOVA_FORCE_CTRL_STRUCT.position_limit_x = 0.5500
KINOVA_FORCE_CTRL_STRUCT.position_limit_y = 0
KINOVA_FORCE_CTRL_STRUCT.position_limit_z = 0

KINOVA_FORCE_CTRL_STRUCT.move_step_x = 0.05
KINOVA_FORCE_CTRL_STRUCT.move_step_y = 0
KINOVA_FORCE_CTRL_STRUCT.move_step_z = 0

KINOVA_FORCE_CTRL_FUNCTION()

A_Sleep(300)

## Step13: Kinova Arm Backward

KINOVA_MANIPULATE_STRUCT.x = --0.015
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 15

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(500)

## Step14: Valve Size Recognition(Using Gripper)

VALVE_SIZE_RECOG_STRUCT.grasp_pose_1 = 1570
VALVE_SIZE_RECOG_STRUCT.grasp_pose_2 = 1570

VALVE_SIZE_RECOG_STRUCT.release_pose_1 = 1950
VALVE_SIZE_RECOG_STRUCT.release_pose_2 = 1950

VALVE_SIZE_RECOG_STRUCT.force_threshold = 90

VALVE_SIZE_RECOG_STRUCT.trial = 18
VALVE_SIZE_RECOG_STRUCT.rotation_angle = 90

VALVE_SIZE_RECOG_STRUCT.retry_num = 1

VALVE_SIZE_RECOG_FUNCTION()

gi_valve_size     = VALVE_SIZE_RECOG_GET_SIZE()
gb_valve_rotation = VALVE_SIZE_RECOG_GET_ANGLE()

A_Sleep(300)

## Step15: Gripper Release

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 2800
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 2800
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = -2

GRIPPER_FORCE_CTRL_FUNCTION()
## Step16: Kinova Arm Back

KINOVA_MANIPULATE_STRUCT.x = --0.08
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(500)

## Step17: Align to Panel
IF(gi_valve_size != -1)

KINOVA_ALIGN_TO_PANEL.do_init_motion = false
KINOVA_ALIGN_TO_PANEL_FUNCTION()


ELSE(GoTo: 0)

##########################################_MISSION_END_##########################################
