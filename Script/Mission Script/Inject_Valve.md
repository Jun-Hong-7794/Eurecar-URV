/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Valve Recognition

## Step0: Grasp 120

GRIPPER_FORCE_CTRL_STRUCT.bend_deg = 120
GRIPPER_FORCE_CTRL_STRUCT.forece_threshold = 1350

GRIPPER_FORCE_CTRL_FUNCTION()

## Step1: Grasp 60

GRIPPER_FORCE_CTRL_STRUCT.bend_deg = 60
GRIPPER_FORCE_CTRL_STRUCT.forece_threshold = 1350

GRIPPER_FORCE_CTRL_FUNCTION()

## Step2: Grasp 80

GRIPPER_FORCE_CTRL_STRUCT.bend_deg = 80
GRIPPER_FORCE_CTRL_STRUCT.forece_threshold = 1350

GRIPPER_FORCE_CTRL_FUNCTION()
A_Sleep(1000)

## Step3: Grasp 20

GRIPPER_FORCE_CTRL_STRUCT.bend_deg = 20
GRIPPER_FORCE_CTRL_STRUCT.forece_threshold = 1350


##########################################_MISSION_END_##########################################
