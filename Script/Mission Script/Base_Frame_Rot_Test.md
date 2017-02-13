/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Base Frame Rot Test

## Step0: Kinova Test

KINOVA_MANIPULATE_STRUCT.x = 0.25
KINOVA_MANIPULATE_STRUCT.y = -0.2992
KINOVA_MANIPULATE_STRUCT.z = 0.3900

KINOVA_MANIPULATE_STRUCT.roll = -1.1
KINOVA_MANIPULATE_STRUCT.pitch = 1.57
KINOVA_MANIPULATE_STRUCT.yaw = 1.57

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(500)

## Step1: Kinova Test

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = ==

KINOVA_MANIPULATE_STRUCT.roll = ++1.1
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(500)

##########################################_MISSION_END_##########################################
