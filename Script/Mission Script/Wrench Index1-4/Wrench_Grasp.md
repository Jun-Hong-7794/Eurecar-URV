/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*
/*<
# Title: Wrench Grasp

global_bool gb_froce_check = false;

global_double gd_standard_x = 0;
global_double gd_standard_y = 0;
global_double gd_standard_z = 0;

## Step11: Kinova Arm Go Right

KINOVA_MANIPULATE_STRUCT.mode = 2

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = --0.12
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = 2.1374
KINOVA_MANIPULATE_STRUCT.pitch = 1.6015
KINOVA_MANIPULATE_STRUCT.yaw = -2.2821

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)

## Step8: Kinova Arm Down 
/*<
IF(gi_valve_size > 19)

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = 0.2807

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)

## Step8: Kinova Arm Down 
IF(gi_valve_size < 22)
/*>
KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = 0.2957

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)

## Step9: Magnet On

GRIPPER_MAGNET_CTRL_STRUCT.fl_magnet = true

GRIPPER_MAGNET_CTRL_FUNCTION()
A_Sleep(1000)

## Step12: Kinova Force Control(Go Forward)

KINOVA_FORCE_CTRL_STRUCT.step_count = 100

KINOVA_FORCE_CTRL_STRUCT.mode = 1

KINOVA_FORCE_CTRL_STRUCT.force_threshold = 0
KINOVA_FORCE_CTRL_STRUCT.force_threshold_x = 9
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
## Step5: LRF-Kinova Backward

LRF_K_VERTICAL_CTRL_STRUCT.mode = 3

LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 270
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(500)

##Step12: Wrench Check!

KINOVA_FORCE_CHECK_STRUCT.force_threshold_x = 0
KINOVA_FORCE_CHECK_STRUCT.force_threshold_y = 0
KINOVA_FORCE_CHECK_STRUCT.force_threshold_z = 6.5

KINOVA_FORCE_CHECK_STRUCT. = check_threshold = 75
KINOVA_FORCE_CHECK_STRUCT.check_count = 100

gb_froce_check = KINOVA_FORCE_CHECK_FUNCTION()


## Step13: Grasp
IF(gb_froce_check)

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 1570
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 1570
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = 350

GRIPPER_FORCE_CTRL_FUNCTION()

A_Sleep(500)

ELSE(GoTo:0)

## Step18: Gripper Release

GRIPPER_GO_TO_REL_POSE_STRUCT.pose_1 = 30
GRIPPER_GO_TO_REL_POSE_STRUCT.pose_2 = 30

GRIPPER_GO_TO_REL_POSE_FUNCTION()

A_Sleep(500)

## Step9: Magnet On

GRIPPER_MAGNET_CTRL_STRUCT.fl_magnet = true
GRIPPER_MAGNET_CTRL_FUNCTION()

A_Sleep(500)

## Step13: Grasp2

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 1570
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 1570
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = 350

GRIPPER_FORCE_CTRL_FUNCTION()

A_Sleep(500)

## Step14: Align to Panel
KINOVA_ALIGN_TO_PANEL.do_init_motion = false
KINOVA_ALIGN_TO_PANEL_FUNCTION()

##########################################_MISSION_END_##########################################
