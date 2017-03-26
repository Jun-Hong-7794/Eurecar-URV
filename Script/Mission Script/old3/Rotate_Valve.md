/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Rotate Valve

## Step17: Rotate Valve(Turn Left)
 
KINOVA_ROTATE_VALVE_STRUCT.using_current_coord = true

KINOVA_ROTATE_VALVE_STRUCT.center_x = 0
KINOVA_ROTATE_VALVE_STRUCT.center_y = 0
KINOVA_ROTATE_VALVE_STRUCT.center_z = 0.1168

KINOVA_ROTATE_VALVE_STRUCT.theta = -380

KINOVA_ROTATE_VALVE_STRUCT.wrench_size = gi_valve_size
KINOVA_ROTATE_VALVE_STRUCT.valve_rotation_angle = -1

KINOVA_ROTATE_VALVE_STRUCT.radius_16mm = 14
KINOVA_ROTATE_VALVE_STRUCT.radius_17mm = 16
KINOVA_ROTATE_VALVE_STRUCT.radius_18mm = 19
KINOVA_ROTATE_VALVE_STRUCT.radius_19mm = 19
KINOVA_ROTATE_VALVE_STRUCT.radius_22mm = 19
KINOVA_ROTATE_VALVE_STRUCT.radius_24mm = 19

KINOVA_ROTATE_VALVE_STRUCT.radius_offset_mm = 1

KINOVA_ROTATE_VALVE_STRUCT.init_angle = false

KINOVA_ROTATE_VALVE_FUNCTION()

## Step1: Release All

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 2800
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 2800
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = -2

GRIPPER_FORCE_CTRL_FUNCTION()

A_Sleep(500)

## Step18: Gripper Release - little

GRIPPER_GO_TO_REL_POSE_STRUCT.pose_1 = 30
GRIPPER_GO_TO_REL_POSE_STRUCT.pose_2 = 30

GRIPPER_GO_TO_REL_POSE_FUNCTION()

A_Sleep(500)

## Step18: Gripper Close - little

GRIPPER_GO_TO_REL_POSE_STRUCT.pose_1 = -30
GRIPPER_GO_TO_REL_POSE_STRUCT.pose_2 = -30

GRIPPER_GO_TO_REL_POSE_FUNCTION()

A_Sleep(500)

## Step13: Grasp

GRIPPER_FORCE_CTRL_STRUCT.pose_1 = 1570
GRIPPER_FORCE_CTRL_STRUCT.pose_2 = 1570
GRIPPER_FORCE_CTRL_STRUCT.force_threshold = -2

GRIPPER_FORCE_CTRL_FUNCTION()

A_Sleep(500)


## Step32: Magnet ON

GRIPPER_MAGNET_CTRL_STRUCT.fl_magnet = true

GRIPPER_MAGNET_CTRL_FUNCTION()

A_Sleep(1000)


## Step32: Magnet OFF

GRIPPER_MAGNET_CTRL_STRUCT.fl_magnet = false

GRIPPER_MAGNET_CTRL_FUNCTION()

A_Sleep(1000)


##########################################_MISSION_END_##########################################
