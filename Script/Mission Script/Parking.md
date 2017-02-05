/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Valve Recognition

## Step0: Vehicle Angle Control(Using LRF)
/* unit is mm
LRF_VEHICLE_ANGLE_CTRL_STRUCT.desired_angle = 0
LRF_VEHICLE_ANGLE_CTRL_STRUCT.error_boundary = 0.5

LRF_VEHICLE_ANGLE_CTRL_STRUCT.s_deg = 45
LRF_VEHICLE_ANGLE_CTRL_STRUCT.e_deg = 135

LRF_VEHICLE_ANGLE_CTRL_STRUCT.velocity = 77

LRF_VEHICLE_ANGLE_CTRL_STRUCT.sensor_option = false

LRF_VEHICLE_ANGLE_CTRL_FUNCTION()

## Step1: Vehicle Horizen Control(Using LRF)
/* unit is mm
LRF_VEHICLE_HORIZEN_CTRL_STRUCT.inlier_distance = 1000

LRF_VEHICLE_HORIZEN_CTRL_STRUCT.desired_avr_inlier_deg = 80.125
LRF_VEHICLE_HORIZEN_CTRL_STRUCT.error_deg_boundary = 2

LRF_VEHICLE_HORIZEN_CTRL_STRUCT.s_deg = 30
LRF_VEHICLE_HORIZEN_CTRL_STRUCT.e_deg = 150


LRF_VEHICLE_HORIZEN_CTRL_STRUCT.velocity = 65

LRF_VEHICLE_HORIZEN_CTRL_STRUCT.sensor_option = false

LRF_VEHICLE_HORIZEN_CTRL_FUNCTION()


##########################################_MISSION_END_##########################################
