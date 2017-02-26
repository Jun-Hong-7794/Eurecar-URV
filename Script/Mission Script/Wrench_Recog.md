/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2

global_int gi_wrench_hanger_index = 3
/*gi_valve_size

# Title: Valve Recognition

## Step0: Align to Panel

KINOVA_ALIGN_TO_PANEL_FUNCTION()

A_Sleep(4000)

## Step1: LRF-Kinova Vertical CTRL(NEW)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 2

LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 240
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 300

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(500)

## Step2: Vehicle Horizen Control(New, Using Kinova-LRF)

LRF_V_HORIZEN_CTRL_STRUCT.mode = 3

LRF_V_HORIZEN_CTRL_STRUCT.desired_h_dst = 850

LRF_V_HORIZEN_CTRL_STRUCT.error = 2

LRF_V_HORIZEN_CTRL_STRUCT.s_deg = 10
LRF_V_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_V_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_V_HORIZEN_CTRL_STRUCT.velocity = 70

LRF_V_HORIZEN_CTRL_STRUCT.loop_sleep = 30

LRF_V_HORIZEN_CTRL_FUNCTION()

A_Sleep(500)

## Step4: KINOVA Angle Control(New, Using Dynamixel Pro-LRF)
/* mode =2 => Left
LRF_K_ANGLE_CTRL_STRUCT.mode = 3

LRF_K_ANGLE_CTRL_STRUCT.error = 0.3
LRF_K_ANGLE_CTRL_STRUCT.desired_angle = 0

LRF_K_ANGLE_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_ANGLE_CTRL_STRUCT.s_deg = 10
LRF_K_ANGLE_CTRL_STRUCT.e_deg = 170

LRF_K_ANGLE_CTRL_STRUCT.unit_deg = 0.3

LRF_K_ANGLE_CTRL_STRUCT.loop_sleep = 30

LRF_K_ANGLE_CTRL_FUNCTION()


## Step3: LRF-Kinova Vertical CTRL(NEW)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 3

LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 240
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(500)

## Step3: LRF-Kinova Horizen CTRL(NEW)

LRF_K_HORIZEN_CTRL_STRUCT.mode = 3

LRF_K_HORIZEN_CTRL_STRUCT.only_sensing_moving = false

LRF_K_HORIZEN_CTRL_STRUCT.desired_h_dst = 850

LRF_K_HORIZEN_CTRL_STRUCT.error = 2

LRF_K_HORIZEN_CTRL_STRUCT.s_deg = 10 
LRF_K_HORIZEN_CTRL_STRUCT.e_deg = 170

LRF_K_HORIZEN_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_HORIZEN_CTRL_STRUCT.loop_sleep = 30 

LRF_K_HORIZEN_CTRL_FUNCTION()

A_Sleep(500)

## Step4: LRF-Kinova Vertical CTRL(NEW)

LRF_K_VERTICAL_CTRL_STRUCT.mode = 3

LRF_K_VERTICAL_CTRL_STRUCT.only_sensing_moving = false

LRF_K_VERTICAL_CTRL_STRUCT.desired_v_dst = 350
LRF_K_VERTICAL_CTRL_STRUCT.error = 2

LRF_K_VERTICAL_CTRL_STRUCT.s_deg = 10
LRF_K_VERTICAL_CTRL_STRUCT.e_deg = 170

LRF_K_VERTICAL_CTRL_STRUCT.inlier_lrf_dst = 1100

LRF_K_VERTICAL_CTRL_STRUCT.loop_sleep = 30

LRF_K_VERTICAL_CTRL_FUNCTION()

A_Sleep(500)

## Step5: Kinova Arm Dw

KINOVA_MANIPULATE_STRUCT.x = ==
KINOVA_MANIPULATE_STRUCT.y = ==
KINOVA_MANIPULATE_STRUCT.z = 0.2438

KINOVA_MANIPULATE_STRUCT.roll = ==
KINOVA_MANIPULATE_STRUCT.pitch = ==
KINOVA_MANIPULATE_STRUCT.yaw = ==

KINOVA_MANIPULATE_STRUCT.force_threshold = 10

KINOVA_MANIPULATE_FUNCTION()

A_Sleep(500)

## Step6: Wrench Recognition

WRENCH_RECOGNITION_STRUCT.num_of_wrench = 6
WRENCH_RECOGNITION_STRUCT.loop_count = 50
WRENCH_RECOGNITION_STRUCT.valve_size = gi_valve_size
WRENCH_RECOGNITION_STRUCT.valve_size = 16
/*gi_wrench_hanger_index = WRENCH_RECOGNITION_FUNCTION()

A_Sleep(2000)

## Step13: Align to Panel
IF(gi_wrench_hanger_index != -1)

KINOVA_ALIGN_TO_PANEL_FUNCTION()

ELSE(GoTo: 0)

##########################################_MISSION_END_##########################################
