/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Valve Recognition

global_int gi_valve_size = 19
global_double gb_valve_rotation = 30

global_double gd_check_v_dst = 30
global_bool gb_check_v_dst_rst = false

## Step0: Rotator
ROTATOR_STRUCT.desired_position = -70000
ROTATOR_STRUCT.fl_rotator_torque = true

ROTATOR_FUNCTION()
A_Sleep(500)

## Step1: Align to Panel
KINOVA_ALIGN_TO_PANEL.do_init_motion = true
KINOVA_ALIGN_TO_PANEL_FUNCTION()

A_Sleep(1000)

## Step2: Rotator
ROTATOR_STRUCT.desired_position = 0
ROTATOR_STRUCT.fl_rotator_torque = true

ROTATOR_FUNCTION()
A_Sleep(2500)

# Step3: Check Vertical Distance
CHECK_CURRENT_V_DISTANCE_STRUCT.mode = 3

CHECK_CURRENT_V_DISTANCE_STRUCT.s_deg = 10
CHECK_CURRENT_V_DISTANCE_STRUCT.e_deg = 170

CHECK_CURRENT_V_DISTANCE_STRUCT.maximum_lrf_dst = 1100
CHECK_CURRENT_V_DISTANCE_STRUCT.desired_v_dst = 290

CHECK_CURRENT_V_DISTANCE_STRUCT.error_bound = 50

gd_check_v_dst = GET_CHECK_CURRENT_V_DISTANCE_BIAS()
gb_check_v_dst_rst = CHECK_CURRENT_V_DISTANCE_FUNCTION()

A_Sleep(500)

## Step4: Rotator
IF(!gb_check_v_dst_rst)

ROTATOR_STRUCT.desired_position = -90000
ROTATOR_STRUCT.fl_rotator_torque = true

ROTATOR_FUNCTION()
A_Sleep(500)

# Step5: Parking Retry
IF(!gb_check_v_dst_rst)

PARKING_RETRY_STRUCT.bias = gd_check_v_dst
PARKING_RETRY_FUNCTION()
A_Sleep(1000)

## Step4: Rotator
IF(!gb_check_v_dst_rst)

ROTATOR_STRUCT.desired_position = 0
ROTATOR_STRUCT.fl_rotator_torque = true

ROTATOR_FUNCTION()
A_Sleep(500)


## Step6: LRF-Kinova Vertical CTRL(NEW)
IF(gb_check_v_dst_rst)

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

ELSE(GoTo:3)

## Step7: Vehicle Horizen Control(New, Using Kinova-LRF)

LRF_V_HORIZEN_CTRL_STRUCT.mode = 2

LRF_V_HORIZEN_CTRL_STRUCT.desired_h_dst = 335

LRF_V_HORIZEN_CTRL_STRUCT.error = 20

LRF_V_HORIZEN_CTRL_STRUCT.s_deg = 10
LRF_V_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_V_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_V_HORIZEN_CTRL_STRUCT.velocity = 65

LRF_V_HORIZEN_CTRL_STRUCT.loop_sleep = 30

LRF_V_HORIZEN_CTRL_FUNCTION()

A_Sleep(300)

## Step8: Gripper Release

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 2800
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 2800
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = -2

GRIPPER_FORCE_CTRL_FUNCTION()

A_Sleep(500)

## Step9: KINOVA Angle Control(New, Using Dynamixel Pro-LRF)
/* mode =2 => Left
LRF_K_ANGLE_CTRL_STRUCT.mode = 2

LRF_K_ANGLE_CTRL_STRUCT.error = 0.3
LRF_K_ANGLE_CTRL_STRUCT.desired_angle = -0.5

LRF_K_ANGLE_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_K_ANGLE_CTRL_STRUCT.e_deg = 170

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 0.3

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()

A_Sleep(300)

## Step10: LRF-Kinova Vertical CTRL(NEW)

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

## Step4: KINOVA Angle Control(New, Using Dynamixel Pro-LRF)
/* mode =2 => Left
LRF_K_ANGLE_CTRL_STRUCT.mode = 2

LRF_K_ANGLE_CTRL_STRUCT.error = 0.3
LRF_K_ANGLE_CTRL_STRUCT.desired_angle = -0.5

LRF_K_ANGLE_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_K_ANGLE_CTRL_STRUCT.e_deg = 170

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 0.3

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()

A_Sleep(300)

## Step4: LRF-Kinova Vertical CTRL(NEW)

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


## Step5: LRF-Kinova Horizen CTRL(NEW)

LRF_K_HORIZEN_CTRL_STRUCT.mode = 1

LRF_K_HORIZEN_CTRL_STRUCT.only_sensing_moving = false

LRF_K_HORIZEN_CTRL_STRUCT.desired_h_dst = 340

LRF_K_HORIZEN_CTRL_STRUCT.error = 2

LRF_K_HORIZEN_CTRL_STRUCT.s_deg = 10 
LRF_K_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_K_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_HORIZEN_CTRL_STRUCT.loop_sleep = 300

LRF_K_HORIZEN_CTRL_FUNCTION()

A_Sleep(500)

## Step6: Kinova Arm Down

KINOVA_MANIPULATE_STRUCT.mode = 2

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = 0.1823
/*KINOVA_MANIPULATE_STRUCT.z = 0.2223

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(1000)

## Step7: Kinova Force Control(Go Forward)

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

## Step8: Kinova Arm Backward

KINOVA_MANIPULATE_STRUCT.x = --0.015
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 15

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(500)

## Step9: Valve Size Recognition(Using Gripper)

VALVE_SIZE_RECOG_STRUCT.grasp_pose_1 = 1570
VALVE_SIZE_RECOG_STRUCT.grasp_pose_2 = 1570

VALVE_SIZE_RECOG_STRUCT.release_pose_1 = 1950
VALVE_SIZE_RECOG_STRUCT.release_pose_2 = 1950

VALVE_SIZE_RECOG_STRUCT.force_threshold = 90

VALVE_SIZE_RECOG_STRUCT.trial = 36
VALVE_SIZE_RECOG_STRUCT.rotation_angle = 180

VALVE_SIZE_RECOG_STRUCT.retry_num = 1

VALVE_SIZE_RECOG_FUNCTION()

gi_valve_size     = VALVE_SIZE_RECOG_GET_SIZE()
gb_valve_rotation = VALVE_SIZE_RECOG_GET_ANGLE()

A_Sleep(300)

## Step10: Gripper Release

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 2800
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 2800
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = -2

GRIPPER_FORCE_CTRL_FUNCTION()
## Step11: Kinova Arm Back

KINOVA_MANIPULATE_STRUCT.x = --0.08
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(500)

## Step13: Align to Panel
IF(gi_valve_size != -1)

KINOVA_ALIGN_TO_PANEL.do_init_motion = false
KINOVA_ALIGN_TO_PANEL_FUNCTION()


ELSE(GoTo: 0)

##########################################_MISSION_END_##########################################
