/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Valve Recognition

## Step0: Vehicle Angle Control(Using LRF)
/* unit is mm
LRF_VEHICLE_ANGLE_CTRL_STRUCT.desired_angle = 0
LRF_VEHICLE_ANGLE_CTRL_STRUCT.error_boundary = 0.5

LRF_VEHICLE_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_VEHICLE_ANGLE_CTRL_STRUCT.e_deg = 180

LRF_VEHICLE_ANGLE_CTRL_STRUCT.velocity = 65


LRF_VEHICLE_ANGLE_CTRL_FUNCTION()

## Step1: Vehicle Horizen Control(Using LRF)
/* unit is mm
LRF_VEHICLE_HORIZEN_CTRL_STRUCT.inlier_distance = 800

LRF_VEHICLE_HORIZEN_CTRL_STRUCT.desired_avr_inlier_deg = 90

LRF_VEHICLE_HORIZEN_CTRL_STRUCT.error_deg_boundary = 3

LRF_VEHICLE_HORIZEN_CTRL_STRUCT.velocity = 65

LRF_VEHICLE_HORIZEN_CTRL_FUNCTION()

## Step2: Vehicle Angle Control(Using LRF)
/* unit is mm
LRF_VEHICLE_ANGLE_CTRL_STRUCT.desired_angle = 0
LRF_VEHICLE_ANGLE_CTRL_STRUCT.error_boundary = 3

LRF_VEHICLE_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_VEHICLE_ANGLE_CTRL_STRUCT.e_deg = 180

LRF_VEHICLE_ANGLE_CTRL_STRUCT.velocity = 65


LRF_VEHICLE_ANGLE_CTRL_FUNCTION()


##########################################_MISSION_END_##########################################