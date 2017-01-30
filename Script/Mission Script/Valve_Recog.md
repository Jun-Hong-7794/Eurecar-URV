/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Valve Recognition

## Step0: Fined Valve Location(Using Segnet)

/* Init Postion
/* Return Valve Rough Position 
/* ex) -KP_kinova_position = SegnetServoing(/*Opetion*/)
/*
/* _if(!-BOOL_my_value) continue_this;
## Step1: Approach Valve(Using Force FeedBack)

/* ex) DoManipulate(-KP_kinova_position, `x_offset`, `y_offset`, `z_offset`)
/* `x` is number

## Step2: Fined Valve Location(Using Force Feedback) & Size Estimation

/* _glob_ -DOUBLE_valve_size =  ValveSizeEstimate()

## Step0: Fined Valve Location(Using Segnet)
B_Sleep(1000)
KINOVA_FORCE_CTRL_STRUCT.step_count = 30
KINOVA_FORCE_CTRL_STRUCT.force_threshold = 10
KINOVA_FORCE_CTRL_STRUCT.move_step_x = 0
KINOVA_FORCE_CTRL_STRUCT.move_step_y = 0.05
KINOVA_FORCE_CTRL_STRUCT.move_step_z = 0.05

KINOVA_FORCE_CTRL_FUNCTION()
A_Sleep(1000)
## Step1: Fined Valve Location(Using Segnet)

KINOVA_FORCE_CTRL_STRUCT.step_count = 30
KINOVA_FORCE_CTRL_STRUCT.force_threshold = 10
KINOVA_FORCE_CTRL_STRUCT.move_step_x = 0
KINOVA_FORCE_CTRL_STRUCT.move_step_y = -0.05
KINOVA_FORCE_CTRL_STRUCT.move_step_z = -0.05

KINOVA_FORCE_CTRL_FUNCTION()

##########################################_MISSION_END_##########################################
