/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2

global_int gi_valve_size = 19
global_double gb_valve_rotation = 0

global_int gi_wrench_hanger_index = 4

# Title: Wrench Recognition

global_int gi_valve_size = 19
global_double gb_valve_rotation = 0

global_double gd_check_v_dst = 30
global_bool gb_check_v_dst_rst = false

## Step1: Vehicle Moving
VEHICLE_LOCALIZATION_ON_PANEL_STRUCT.desired_h_dst = 0.9
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

##########################################_MISSION_END_##########################################
