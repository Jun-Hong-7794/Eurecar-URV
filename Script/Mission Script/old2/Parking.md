/* Eurecar Scenario Script 
/* Made By Jun Hong 28.01.2017
/* This Script is for 2107 MBZIRC Challenge2
/*

# Title: Parking

## Step0: Panel Detection

VEHICLE_PARKING_FUNCTION()

## Step1: Parking Retry Check
PARKING_RETRY_STRUCT.max_dst = 1.2
PARKING_RETRY_STRUCT.min_dst = 0.90

PARKING_RETRY_STRUCT.desired_dst = 1.05

PARKING_RETRY_FUNCTION()

##########################################_MISSION_END_##########################################
