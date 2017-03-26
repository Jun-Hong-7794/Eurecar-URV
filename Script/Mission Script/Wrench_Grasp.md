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

## Step0: Align to Panel

KINOVA_ALIGN_TO_PANEL.do_init_motion = false
KINOVA_ALIGN_TO_PANEL_FUNCTION()

A_Sleep(500)

## Step7: Kinova Arm Go Right

KINOVA_MANIPULATE_STRUCT.mode = 2

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = --0.10
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = 2.1374
KINOVA_MANIPULATE_STRUCT.pitch = 1.6015
KINOVA_MANIPULATE_STRUCT.yaw = -2.2821

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)

## Step5: LRF-Kinova Go Forward

LRF_K_VERTICAL_CTRL_STRUCT.mode = 3

LRF_K_VERTICAL_CTRL_STRUCT.force_option = true

LRF_K_VERTICAL_CTRL_STRUCT.force_x = 8
LRF_K_VERTICAL_CTRL_STRUCT.force_y = 0
LRF_K_VERTICAL_CTRL_STRUCT.force_z = 0

LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = true

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 110
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(2000)

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

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 1570
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 1570
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = 350

GRIPPER_FORCE_CTRL_FUNCTION()

A_Sleep(500)

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
