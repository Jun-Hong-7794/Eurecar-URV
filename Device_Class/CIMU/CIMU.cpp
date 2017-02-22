#include "CIMU.h"

u8 enable_data_stats_output = 0;

//The primary device interface structure
mip_interface device_interface;

//Packet Counters (valid, timeout, and checksum errors)
u32 filter_valid_packet_count  = 0;
u32 ahrs_valid_packet_count = 0;

u32 filter_timeout_packet_count  = 0;
u32 ahrs_timeout_packet_count = 0;

u32 filter_checksum_error_packet_count  = 0;
u32 ahrs_checksum_error_packet_count = 0;

//Example data field storage

//AHRS
mip_ahrs_scaled_gyro  curr_ahrs_gyro;
mip_ahrs_scaled_accel curr_ahrs_accel;
mip_ahrs_scaled_mag   curr_ahrs_mag;
mip_ahrs_internal_timestamp curr_timestamp;

mip_ahrs_delta_theta curr_ahrs_delta_theta;
mip_ahrs_euler_angles curr_angles;

mip_ahrs_delta_velocity curr_ahrs_delta_velocity;

//FILTER
mip_filter_attitude_euler_angles curr_filter_angles;


CIMU::CIMU()
{

}

CIMU::~CIMU()
{

}

bool CIMU::IMUInit(string _comport, double _init_heading)
{
    // Set yaw bias -----------
    if(_init_heading > 180.0)
    {
        _init_heading = _init_heading - 360.0;
    }
    else
    {
        _init_heading = _init_heading;
    }

    yaw_bias = _init_heading/180.0*3.1415926535;
    // ------------------------

    com_port = atoi(_comport.c_str());

    baudrate = 115200;

    if(mip_interface_init(com_port,baudrate,&device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
    {
        cout<<"IMU Init failed!" << endl;
        return false;
    }
    else
    {
        cout<<"IMU Init succeed!" << endl;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Standard Mode Tests
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    device_descriptors_size  = 128*2;
    com_mode = MIP_SDK_GX4_25_IMU_STANDARD_MODE;

    cout<<"----------------------------------------------------------------------\n"<<endl;
    cout<<"Putting Device Into Standard Mode\n"<<endl;
    cout<<"----------------------------------------------------------------------\n\n"<<endl;

    //Set communication mode
    while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK){}

    //Verify device mode setting
    while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK){}


    ///
    //Put the GX4-25 into idle mode
    ///

    cout<<"----------------------------------------------------------------------\n"<<endl;
    cout<<"Idling Device\n"<<endl;
    cout<<"----------------------------------------------------------------------\n\n"<<endl;

    while(mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK){}



    //Set device to external heading update mode
    heading_source = 0x0;

    while(mip_filter_heading_source(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &heading_source) != MIP_INTERFACE_OK){}



//    ///
//    //Capture Gyro Bias
//    ///

//    cout << "----------------------------------------------------------------------\n" << endl;
//    cout << "Performing Gyro Bias capture.\nPlease keep device stationary during the 5 second gyro bias capture interval\n" << endl;
//    cout << "----------------------------------------------------------------------\n\n" << endl;

//    duration = 5000; //milliseconds

//    while(mip_3dm_cmd_capture_gyro_bias(&device_interface, duration, bias_vector) != MIP_INTERFACE_OK){}

//    printf("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1], bias_vector[2]);


//    cout << "----------------------------------------------------------------------" << endl;
//    cout << "Setting Gyro Bias Vector\n" << endl;
//    cout << "----------------------------------------------------------------------" << endl;


//    while(mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, bias_vector) != MIP_INTERFACE_OK){}



    cout<<"Re-initializing filter (required for tare)\n\n"<<endl;

    //Re-initialize the filter with euler_angle
    angles[0] = angles[1] = angles[2] = 0;

    while(mip_filter_set_init_attitude(&device_interface, angles) != MIP_INTERFACE_OK){}

//    // Re-initialize the filter with magnetometer
//    float declination = 0.0;

//    while(mip_filter_set_init_attitude_from_ahrs(&device_interface, declination) != MIP_INTERFACE_OK) {}

//    cout << "magneto init finshed!" << endl;
    //Wait for Filter to re-establish running state
    Sleep(5000);


    ///
    //Tare Orientation
    ///

    cout<<"----------------------------------------------------------------------\n"<<endl;
    cout<<"Performing Tare Orientation Command\n"<<endl;
    cout<<"----------------------------------------------------------------------\n\n"<<endl;


    //Cycle through axes combinations
    for(i=1; i<8; i++)
    {
        if(mip_filter_tare_orientation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, i) != MIP_INTERFACE_OK)
        {
            cout<<"ERROR: Failed Axis - "<<endl;

             if(i & FILTER_TARE_ROLL_AXIS)
                 cout<<" Roll Axis "<<endl;

             if(i & FILTER_TARE_PITCH_AXIS)
                 cout<<" Pitch Axis "<<endl;

             if(i & FILTER_TARE_YAW_AXIS)
                 cout<<" Yaw Axis "<<endl;
        }
        else
        {
             cout<<"Tare Configuration = "<< i <<"\n"<<endl;

             cout<<"Tared -"<<endl;

             if(i & FILTER_TARE_ROLL_AXIS)
                 cout<<" Roll Axis "<<endl;

             if(i & FILTER_TARE_PITCH_AXIS)
                 cout<<" Pitch Axis "<<endl;

             if(i & FILTER_TARE_YAW_AXIS)
                 cout<<" Yaw Axis "<<endl;

             cout<<"\n\n"<<endl;
        }

        Sleep(100);
    }


    ///
    //Setup the GX4-25 dataset callbacks
    ///

    if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_FILTER_DATA_SET, NULL, &FilterPacketCallback) != MIP_INTERFACE_OK)
        return false;

    if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &AhrsPacketCallback) != MIP_INTERFACE_OK)
        return false;


    //Enable the output of data statistics
    enable_data_stats_output = 1;

    #ifdef FILTER_MODE
    ///
    //Setup the FILTER datastream format
    ///

    cout<<"----------------------------------------------------------------------\n"<<endl;
    cout<<"Setting the Estimation Filter datastream format\n"<<endl;
    cout<<"----------------------------------------------------------------------\n\n"<<endl;

    data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_EULER_ANGLES;

    data_stream_format_decimation[0]  = 0x32;

    data_stream_format_num_entries = 1;

    while(mip_3dm_cmd_filter_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
                          data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}


    #else
    ///
    //Setup the AHRS datastream format
    ///


    cout<<"----------------------------------------------------------------------\n"<<endl;
    cout<<"Setting the AHRS message format\n"<<endl;
    cout<<"----------------------------------------------------------------------\n\n"<<endl;

    data_stream_format_descriptors[0] = MIP_AHRS_DATA_EULER_ANGLES;
    data_stream_format_descriptors[1] = MIP_AHRS_DATA_ACCEL_SCALED;

    data_stream_format_decimation[0]  = 0x32;
    data_stream_format_decimation[1]  = 0x32;

    data_stream_format_num_entries = 2;

    while(mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
    #endif


    cout<<"*************IMU Initialization finished!************"<<endl;

    imu_init = true;
    return true;

}

void AhrsPacketCallback(void *_user_ptr, u8 *_packet, u16 _packet_size, u8 _callback_type)
{

    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch(_callback_type)
    {
    ///
    //Handle valid packets
    ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    {
        ahrs_valid_packet_count++;

        ///
        //Loop through all of the data fields
        ///

        while(mip_get_next_field(_packet, &field_header, &field_data, &field_offset) == MIP_OK)
        {

            ///
            // Decode the field
            ///

            switch(field_header->descriptor)
            {
            ///
            // Scaled Accelerometer
            ///

            case MIP_AHRS_DATA_ACCEL_SCALED:
            {
                memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

                //For little-endian targets, byteswap the data field
                mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

            }break;

            ///
            // Scaled Gyro
            ///

            case MIP_AHRS_DATA_GYRO_SCALED:
            {
                memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

                //For little-endian targets, byteswap the data field
                mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

            }break;

            ///
            // Scaled Magnetometer
            ///

            case MIP_AHRS_DATA_MAG_SCALED:
            {
                memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

                //For little-endian targets, byteswap the data field
                mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

            }break;

            case MIP_AHRS_DATA_DELTA_THETA:
            {

                memcpy(&curr_ahrs_delta_theta, field_data, sizeof(mip_ahrs_delta_theta));

                //For little-endian targets, byteswap the data field
                mip_ahrs_delta_theta_byteswap(&curr_ahrs_delta_theta);

            }break;

            case MIP_AHRS_DATA_EULER_ANGLES:
            {

                memcpy(&curr_angles, field_data, sizeof(mip_ahrs_euler_angles));

                //For little-endian targets, byteswap the data field
                mip_ahrs_euler_angles_byteswap(&curr_angles);

            }break;

            case MIP_AHRS_DATA_TIME_STAMP_INTERNAL:
            {

                memcpy(&curr_timestamp, field_data, sizeof(mip_ahrs_internal_timestamp));

                //For little-endian targets, byteswap the data field
                mip_ahrs_internal_timestamp_byteswap(&curr_timestamp);

            }break;

            case MIP_AHRS_DATA_DELTA_VELOCITY:
            {
                memcpy(&curr_ahrs_delta_velocity, field_data, sizeof(mip_ahrs_delta_velocity));

                //For little-endian targets, byteswap the data field
                mip_ahrs_delta_velocity_byteswap(&curr_ahrs_delta_velocity);

            }break;

            default: break;
            }
        }
     }break;

     ///
     //Handle checksum error packets
     ///

     case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
     {
        ahrs_checksum_error_packet_count++;
     }break;

     ///
     //Handle timeout packets
     ///

     case MIP_INTERFACE_CALLBACK_TIMEOUT:
     {
        ahrs_timeout_packet_count++;
        }break;
        default: break;
    }
}

void FilterPacketCallback(void *_user_ptr, u8 *_packet, u16 _packet_size, u8 _callback_type)
{
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch(_callback_type)
    {
    ///
    //Handle valid packets
    ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    {
        filter_valid_packet_count++;

        ///
        //Loop through all of the data fields
        ///

        while(mip_get_next_field(_packet, &field_header, &field_data, &field_offset) == MIP_OK)
        {

        ///
        // Decode the field
        ///

        switch(field_header->descriptor)
        {
        ///
        // Estimated Attitude, Euler Angles
        ///

        case MIP_FILTER_DATA_ATT_EULER_ANGLES:
        {
            memcpy(&curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));

            //For little-endian targets, byteswap the data field
            mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);

        }break;

       default: break;
       }
       }
    }break;


    ///
    //Handle checksum error packets
    ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    {
        filter_checksum_error_packet_count++;
    }break;

    ///
    //Handle timeout packets
    ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT:
    {
        filter_timeout_packet_count++;
    }break;
    default: break;
    }
}

u16 CIMU::Mip3dmCmdHwSpecificDeviceStatus(mip_interface *_device_interface, u16 _model_number, u8 _status_selector, u8 *_response_buffer)
{
    gx4_25_basic_status_field *basic_ptr;
    gx4_25_diagnostic_device_status_field *diagnostic_ptr;
    u16 response_size = MIP_FIELD_HEADER_SIZE;

    if(_status_selector == GX4_25_BASIC_STATUS_SEL)
        response_size += sizeof(gx4_25_basic_status_field);
    else if(_status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
        response_size += sizeof(gx4_25_diagnostic_device_status_field);

    while(mip_3dm_cmd_device_status(_device_interface, _model_number, _status_selector, _response_buffer, &response_size) != MIP_INTERFACE_OK){}

    if(_status_selector == GX4_25_BASIC_STATUS_SEL)
    {
        if(response_size != sizeof(gx4_25_basic_status_field))
            return MIP_INTERFACE_ERROR;
        else if(MIP_SDK_CONFIG_BYTESWAP)
        {
            basic_ptr = (gx4_25_basic_status_field *)_response_buffer;

            byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
            byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
            byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
        }

    }
    else if(_status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
    {
        if(response_size != sizeof(gx4_25_diagnostic_device_status_field))
            return MIP_INTERFACE_ERROR;
        else if(MIP_SDK_CONFIG_BYTESWAP)
        {
            diagnostic_ptr = (gx4_25_diagnostic_device_status_field *)_response_buffer;

            byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
            byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
            byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
            byteswap_inplace(&diagnostic_ptr->imu_dropped_packets, sizeof(diagnostic_ptr->imu_dropped_packets));
            byteswap_inplace(&diagnostic_ptr->filter_dropped_packets, sizeof(diagnostic_ptr->filter_dropped_packets));
            byteswap_inplace(&diagnostic_ptr->com1_port_bytes_written, sizeof(diagnostic_ptr->com1_port_bytes_written));
            byteswap_inplace(&diagnostic_ptr->com1_port_bytes_read, sizeof(diagnostic_ptr->com1_port_bytes_read));
            byteswap_inplace(&diagnostic_ptr->com1_port_write_overruns, sizeof(diagnostic_ptr->com1_port_write_overruns));
            byteswap_inplace(&diagnostic_ptr->com1_port_read_overruns, sizeof(diagnostic_ptr->com1_port_read_overruns));
            byteswap_inplace(&diagnostic_ptr->imu_parser_errors, sizeof(diagnostic_ptr->imu_parser_errors));
            byteswap_inplace(&diagnostic_ptr->imu_message_count, sizeof(diagnostic_ptr->imu_message_count));
            byteswap_inplace(&diagnostic_ptr->imu_last_message_ms, sizeof(diagnostic_ptr->imu_last_message_ms));
        }
    }
    else
        return MIP_INTERFACE_ERROR;

    return MIP_INTERFACE_OK;
}

u16 CIMU::Mip3dmCmdHwSpecificImuDeviceStatus(mip_interface *_device_interface, u16 _model_number, u8 _status_selector, u8 *_response_buffer)
{
    gx4_imu_basic_status_field *basic_ptr;
    gx4_imu_diagnostic_device_status_field *diagnostic_ptr;
    u16 response_size = MIP_FIELD_HEADER_SIZE;

    if(_status_selector == GX4_IMU_BASIC_STATUS_SEL)
        response_size += sizeof(gx4_imu_basic_status_field);
    else if(_status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
        response_size += sizeof(gx4_imu_diagnostic_device_status_field);

    while(mip_3dm_cmd_device_status(_device_interface, _model_number, _status_selector, _response_buffer, &response_size) != MIP_INTERFACE_OK){}

    if(_status_selector == GX4_IMU_BASIC_STATUS_SEL)
    {
        if(response_size != sizeof(gx4_imu_basic_status_field))
            return MIP_INTERFACE_ERROR;
        else if(MIP_SDK_CONFIG_BYTESWAP)
        {
            basic_ptr = (gx4_imu_basic_status_field *)_response_buffer;

            byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
            byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
            byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
        }
    }
    else if(_status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
    {

        if(response_size != sizeof(gx4_imu_diagnostic_device_status_field))
            return MIP_INTERFACE_ERROR;
        else if(MIP_SDK_CONFIG_BYTESWAP)
        {
            diagnostic_ptr = (gx4_imu_diagnostic_device_status_field *)_response_buffer;

            byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
            byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
            byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
            byteswap_inplace(&diagnostic_ptr->gyro_range, sizeof(diagnostic_ptr->gyro_range));
            byteswap_inplace(&diagnostic_ptr->mag_range, sizeof(diagnostic_ptr->mag_range));
            byteswap_inplace(&diagnostic_ptr->pressure_range, sizeof(diagnostic_ptr->pressure_range));
            byteswap_inplace(&diagnostic_ptr->temp_degc, sizeof(diagnostic_ptr->temp_degc));
            byteswap_inplace(&diagnostic_ptr->last_temp_read_ms, sizeof(diagnostic_ptr->last_temp_read_ms));
            byteswap_inplace(&diagnostic_ptr->num_gps_pps_triggers, sizeof(diagnostic_ptr->num_gps_pps_triggers));
            byteswap_inplace(&diagnostic_ptr->last_gps_pps_trigger_ms, sizeof(diagnostic_ptr->last_gps_pps_trigger_ms));
            byteswap_inplace(&diagnostic_ptr->dropped_packets, sizeof(diagnostic_ptr->dropped_packets));
            byteswap_inplace(&diagnostic_ptr->com_port_bytes_written, sizeof(diagnostic_ptr->com_port_bytes_written));
            byteswap_inplace(&diagnostic_ptr->com_port_bytes_read, sizeof(diagnostic_ptr->com_port_bytes_read));
            byteswap_inplace(&diagnostic_ptr->com_port_write_overruns, sizeof(diagnostic_ptr->com_port_write_overruns));
            byteswap_inplace(&diagnostic_ptr->com_port_read_overruns, sizeof(diagnostic_ptr->com_port_read_overruns));
        }
    }
    else
        return MIP_INTERFACE_ERROR;

    return MIP_INTERFACE_OK;
}

vector<double> CIMU::GetEulerAngles()
{

#ifdef FILTER_MODE

    mtx_imu.lock();

    while(mip_3dm_cmd_poll_filter(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors)){}
    Sleep(10);

    imu_roll = curr_filter_angles.roll;
    imu_pitch = curr_filter_angles.pitch;
    imu_yaw = curr_filter_angles.yaw + yaw_bias;

    mtx_imu.unlock();

#else
    while(mip_3dm_cmd_poll_ahrs(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK){}
    imu_roll = curr_angles.roll;
    imu_pitch = curr_angles.pitch;
    imu_yaw = curr_angles.yaw;
#endif

    vector<double> return_vec;
    return_vec.push_back(imu_roll);
    return_vec.push_back(imu_pitch);
    return_vec.push_back(imu_yaw);


    return return_vec;
}

bool CIMU::IsIMUInit()
{
    return imu_init;
}
