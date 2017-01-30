/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Valve Recognition

## Step0: Fined Valve Location(Using Segnet)

GRIPPER_FORCE_CTRL_STRUCT.bend_deg = 110
GRIPPER_FORCE_CTRL_STRUCT.forece_threshold = 1350

GRIPPER_FORCE_CTRL_FUNCTION()
A_Sleep(500)
## Step1: Fined Valve Location(Using Segnet)

KINOVA_MANIPULATE_STRUCT.x = 0.2096
KINOVA_MANIPULATE_STRUCT.y = -0.263
KINOVA_MANIPULATE_STRUCT.z = 0.477

KINOVA_MANIPULATE_STRUCT.roll = -0.0264 
KINOVA_MANIPULATE_STRUCT.pitch = 1.1472
KINOVA_MANIPULATE_STRUCT.yaw = 1.5508

KINOVA_MANIPULATE_STRUCT.force_threshold = 13

KINOVA_MANIPULATE_FUNCTION()
A_Sleep(500)

## Step0: Fined Valve Location(Using Segnet)

GRIPPER_FORCE_CTRL_STRUCT.bend_deg = 50
GRIPPER_FORCE_CTRL_STRUCT.forece_threshold = 1350

GRIPPER_FORCE_CTRL_FUNCTION()

## Step1: Fined Valve Location(Using Segnet)

KINOVA_MANIPULATE_STRUCT.x = 0.2484
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = 0.3507

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 13

KINOVA_MANIPULATE_FUNCTION()


##########################################_MISSION_END_##########################################
