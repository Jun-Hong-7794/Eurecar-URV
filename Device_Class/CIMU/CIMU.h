#ifndef IMU_H
#define IMU_H

#include <QThread>
#include <QMutex>
#include <QTimer>
#include <QElapsedTimer>


#include "3DM-GX/mip_sdk.h"
#include "3DM-GX/byteswap_utilities.h"
#include "3DM-GX/mip_gx4_imu.h"
#include "3DM-GX/mip_gx4_25.h"
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <vector>
#include <sys/time.h>
#define MIP_SDK_GX4_25_IMU_STANDARD_MODE  0x01
#define MIP_SDK_GX4_25_IMU_DIRECT_MODE	  0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

#define Sleep(x) usleep(x*1000.0)

#define FILTER_MODE

using namespace std;


void FilterPacketCallback(void *_user_ptr, u8 *_packet, u16 _packet_size, u8 _callback_type);
void AhrsPacketCallback(void *_user_ptr, u8 *_packet, u16 _packet_size, u8 _callback_type);

typedef struct LINEAR_ACCEL_STRUCT{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    u16 valid_flag;
}linear_accel_struct;

class CIMU: public QThread{
    Q_OBJECT

protected:
    void run();

public:
    CIMU();
    ~CIMU();

    bool IMUInit(string _comport, double _init_heading);
    void GetEulerAngles();
    bool IsIMUInit();
    void IMUClose();

    QMutex mtx_imu;

private:


    bool imu_init = false;
    //--------------------------------------------------
    //                 IMU variable
    //

    u32 com_port, baudrate;
    base_device_info_field device_info;
    u8  temp_string[20] = {0};
    u32 bit_result;
    u8  enable = 1;
    u8  data_stream_format_descriptors[10];
    u16 data_stream_format_decimation[10];
    u8  data_stream_format_descriptors_ahrs[10];
    u16 data_stream_format_decimation_ahrs[10];

    u8  data_stream_format_num_entries = 0;
    u8  data_stream_format_num_entries_ahrs = 0;

    u8  readback_data_stream_format_descriptors[10] = {0};
    u16 readback_data_stream_format_decimation[10]  = {0};
    u8  readback_data_stream_format_num_entries     =  0;
    u16 base_rate = 0;
    u16 device_descriptors[128]  = {0};
    u16 device_descriptors_size  = 128*2;
    s16 i;
    u16 j;
    u8  com_mode = 0;
    u8  readback_com_mode = 0;
    float angles[3]             = {0};
    float readback_angles[3]    = {0};
    float offset[3]             = {0};
    float readback_offset[3]    = {0};
    float hard_iron[3]          = {0};
    float hard_iron_readback[3] = {0};
    float soft_iron[9]          = {0};
    float soft_iron_readback[9] = {0};
    u16 estimation_control   = 0, estimation_control_readback = 0;
    u8  heading_source = 0;
    u8  auto_init      = 0;
    float noise[3]          = {0};
    float readback_noise[3] = {0};
    float beta[3]                 = {0};
    float readback_beta[3]        = {0};
    mip_low_pass_filter_settings filter_settings;
    float bias_vector[3]		   = {0};
    u16 duration = 0;


    gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
    gx4_imu_basic_status_field imu_basic_field;
    gx4_25_diagnostic_device_status_field diagnostic_field;
    gx4_25_basic_status_field basic_field;
    mip_filter_external_heading_update_command external_heading_update;
    mip_filter_zero_update_command zero_update_control, zero_update_readback;
    mip_filter_external_heading_with_time_command external_heading_with_time;
    mip_complementary_filter_settings comp_filter_command, comp_filter_readback;

    u8  declination_source_command, declination_source_readback;

    mip_filter_accel_magnitude_error_adaptive_measurement_command        accel_magnitude_error_command, accel_magnitude_error_readback;
    mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
    mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;
    u8 reference_position_enable_command, reference_position_enable_readback;
    double reference_position_command[3], reference_position_readback[3];
    //--------------------------------------------------

    double imu_roll = 0.0;
    double imu_pitch = 0.0;
    double imu_yaw = 0.0;

    double yaw_bias = 0.0;

    //Hardware specific status functions
    u16 Mip3dmCmdHwSpecificDeviceStatus(mip_interface *_device_interface, u16 _model_number, u8 _status_selector, u8 *_response_buffer);
    u16 Mip3dmCmdHwSpecificImuDeviceStatus(mip_interface *_device_interface, u16 _model_number, u8 _status_selector, u8 *_response_buffer);

    u8 enable_data_stats_output = 0;

signals:
    void SignalIMUEuler(vector<double>);
    void SignalIMULinearAccel(mip_filter_linear_acceleration);
    void SignalIMUTimestamp(mip_ahrs_internal_timestamp);
    void SignalIMUDeltaVelocity(mip_ahrs_delta_velocity);
};







#endif // IMU_H
