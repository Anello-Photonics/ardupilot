/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
    driver for Anello X3 IMU system
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ANELLOX3_ENABLED

#include "AP_ExternalAHRS_AnelloX3.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_AnelloX3::AP_ExternalAHRS_AnelloX3(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Anello X3 ExternalAHRS no UART");
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_AnelloX3::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Anello X3 failed to allocate ExternalAHRS update thread");
    }

    hal.scheduler->delay(5000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Anello X3 ExternalAHRS initialised");
}

void AP_ExternalAHRS_AnelloX3::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        build_packet();
        hal.scheduler->delay_microseconds(100);
    }
}



// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_AnelloX3::build_packet()
{
    if (uart == nullptr) {
        return;
    }
    
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        DescriptorSet descriptor;
        if (handle_byte(b, descriptor)) {
            switch (descriptor) {
            case DescriptorSet::IMUData:
                post_imu();
                break;
        }
    }
    }
}

bool AP_ExternalAHRS_AnelloX3::handle_byte(const uint8_t b, DescriptorSet& descriptor)
{
    switch (message_in.state) {
        case ParseState::WaitingFor_SyncOne:
            if (b == SYNC_ONE) {
                message_in.packet.header[0] = b;
                message_in.state = ParseState::WaitingFor_SyncTwo;
            }
            break;
        case ParseState::WaitingFor_SyncTwo:
            if (b == SYNC_TWO) {
                message_in.packet.header[1] = b;
                message_in.state = ParseState::WaitingFor_Descriptor;
            } else {
                message_in.state = ParseState::WaitingFor_SyncOne;
            }
            break;
        case ParseState::WaitingFor_Descriptor:
            message_in.packet.descriptor_set(b);
            message_in.state = ParseState::WaitingFor_PayloadLength;
            break;
        case ParseState::WaitingFor_PayloadLength:
            message_in.packet.payload_length(b);
            message_in.state = ParseState::WaitingFor_Data;
            message_in.index = 0;
            break;
        case ParseState::WaitingFor_Data:
            message_in.packet.payload[message_in.index++] = b;
            if (message_in.index >= message_in.packet.payload_length()) {
                message_in.state = ParseState::WaitingFor_Checksum;
                message_in.index = 0;
            }
            break;
        case ParseState::WaitingFor_Checksum:
            message_in.packet.checksum[message_in.index++] = b;
            if (message_in.index >= 2) {
                message_in.state = ParseState::WaitingFor_SyncOne;
                message_in.index = 0;

                if (valid_packet(message_in.packet)) {
                    descriptor = handle_packet(message_in.packet);
                    return true;
                }
            }
            break;
        }
    return false;
}


// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_AnelloX3::post_imu() const
{
    // note that we will have to find a way to post as two separate INSs
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = imu_data.mems_accel;
        state.gyro = imu_data.mems_gyro;

        state.have_quaternion = false;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: imu_data.mems_accel,
            gyro: imu_data.mems_gyro,
            temperature: -300 // update this later after confirming these post
        };
        AP::ins().handle_external(ins);
    }

    // include mag data once we have shown that we can transmit imu data

}

int8_t AP_ExternalAHRS_AnelloX3::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// Get model/type name
const char* AP_ExternalAHRS_AnelloX3::get_name() const
{
    return "Anello X3";
}

bool AP_ExternalAHRS_AnelloX3::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_imu_pkt < 40);
}

bool AP_ExternalAHRS_AnelloX3::initialised(void) const
{
    return last_imu_pkt != 0;
}

bool AP_ExternalAHRS_AnelloX3::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Anello X3 unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_AnelloX3::get_filter_status(nav_filter_status &status) const
{
    // dummy function for now
}

// get variances
bool AP_ExternalAHRS_AnelloX3::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    // dummy function for now
    return false;
}


// get variances
uint8_t AP_ExternalAHRS_AnelloX3::num_gps_sensors(void) const
{
    // dummy function for now
    return 0;
}

bool AP_ExternalAHRS_AnelloX3::valid_packet(const AnelloX3_Packet & packet)
{
    uint8_t checksum_one = 0;
    uint8_t checksum_two = 0;

    for (int i = 2; i < 4; i++) {
        checksum_one += packet.header[i];
        checksum_two += checksum_one;
    }

    for (int i = 0; i < packet.payload_length(); i++) {
        checksum_one += packet.payload[i];
        checksum_two += checksum_one;
    }

    return packet.checksum[0] == checksum_one && packet.checksum[1] == checksum_two;
}

AP_ExternalAHRS_AnelloX3::DescriptorSet AP_ExternalAHRS_AnelloX3::handle_packet(const AnelloX3_Packet& packet)
{
    const DescriptorSet descriptor = packet.descriptor_set();
    switch (descriptor) {
    case DescriptorSet::IMUData:
        handle_imu(packet);
        break;
    }
    return descriptor;
}

void AP_ExternalAHRS_AnelloX3::handle_imu(const AnelloX3_Packet& packet)
{
    // unpacking function for the X3 IMU messages

    // storage for parsing out raw data payload
    AnelloX3_BinaryPayload bin_payload;

    // keep track of last recv packet
    last_imu_pkt = AP_HAL::millis(); 

    for (int i=0; i < packet.payload_length(); i++) {
        if (i >= 0 && i < 8) {
            // mcu time section, 8 bytes
            bin_payload.mcu_time |= packet.payload[i] << i;
        }
        if (i >= 8 && i < 16) {
            // sync time section, 8 bytes
            bin_payload.sync_time |= packet.payload[i] << (i - 8);
        }
        if (i >= 16 && i < 18) {
            // ax1 section, 2 bytes
            bin_payload.ax1 |= packet.payload[i] << (i - 16);
        }
        if (i >= 18 && i < 20) {
            // ay1 section, 2 bytes
            bin_payload.ay1 |= packet.payload[i] << (i - 18);
        }
        if (i >= 20 && i < 22) {
            // az1 section, 2 bytes
            bin_payload.az1 |= packet.payload[i] << (i - 20);
        }
        if (i >= 22 && i < 24) {
            // wx1 section, 2 bytes
            bin_payload.wx1 |= packet.payload[i] << (i - 22);
        }
        if (i >= 24 && i < 26) {
            // wy1 section, 2 bytes
            bin_payload.wy1 |= packet.payload[i] << (i - 24);
        }
        if (i >= 26 && i < 28) {
            // wz1 section, 2 bytes
            bin_payload.wz1 |= packet.payload[i] << (i - 26);
        }
        if (i >= 28 && i < 32) {
            // og_wx section, 4 bytes
            bin_payload.og_wx |= packet.payload[i] << (i - 28);
        }
        if (i >= 32 && i < 36) {
            // og_wy section, 4 bytes
            bin_payload.og_wy |= packet.payload[i] << (i - 32);
        }
        if (i >= 36 && i < 40) {
            // og_wz section, 4 bytes
            bin_payload.og_wz |= packet.payload[i] << (i - 36);
        }
        if (i >= 40 && i < 42) {
            // mag_x section, 2 bytes
            bin_payload.mag_x |= packet.payload[i] << (i - 40);
        }
        if (i >= 42 && i < 44) {
            // mag_y section, 2 bytes
            bin_payload.mag_y |= packet.payload[i] << (i - 42);
        }
        if (i >= 44 && i < 46) {
            // mag_z section, 2 bytes
            bin_payload.mag_z |= packet.payload[i] << (i - 44);
        }
        if (i >= 46 && i < 48) {
            // temp section, 2 bytes
            bin_payload.temp |= packet.payload[i] << (i - 46);
        }
        if (i >= 48 && i < 50) {
            // mems_ranges section, 2 bytes
            bin_payload.mems_ranges |= packet.payload[i] << (i - 48);
        }
        if (i >= 50 && i < 52) {
            // fog_range section, 2 bytes
            bin_payload.fog_range |= packet.payload[i] << (i - 50);
        }
        if (i >= 52 && i < 53) {
            // fusion_status_x section, 1 byte (?)
            bin_payload.fusion_status_x |= packet.payload[i] << (i - 52);
        }
        if (i >= 53 && i < 54) {
            // fusion_status_y section, 1 byte (?)
            bin_payload.fusion_status_y |= packet.payload[i] << (i - 53);
        }
        if (i >= 54 && i < 55) {
            // fusion_status_z section, 1 byte (?)
            bin_payload.fusion_status_z |= packet.payload[i] << (i - 54);
        }
    }

    // convert the binary data to actual values
   convert_imu_data(bin_payload);
    
}

// convert the binary data to actual values
void AP_ExternalAHRS_AnelloX3::convert_imu_data(const AnelloX3_BinaryPayload& bin_payload)
{

    // mems ranges: gggg gggg ggga aaaa where 'g' is a gyro bit and 'a' is an acc bit

    // parse out ranges
    imu_data.mems_acc_range = bin_payload.mems_ranges & 0x1F; // acc range mask
    imu_data.mems_gyro_range = (bin_payload.mems_ranges >> 5); // gyro range shift
    imu_data.fog_gyro_range = bin_payload.fog_range;

    // calculate mems acc data
    imu_data.mems_accel.x = bin_payload.ax1 * imu_data.mems_acc_range * 3.05e-5;
    imu_data.mems_accel.y = bin_payload.ay1 * imu_data.mems_acc_range * 3.05e-5;
    imu_data.mems_accel.z = bin_payload.az1 * imu_data.mems_acc_range * 3.05e-5;

    // calculate mems gyro data
    imu_data.mems_gyro.x = bin_payload.wx1 * imu_data.mems_gyro_range * 3.5e-5;
    imu_data.mems_gyro.y = bin_payload.wy1 * imu_data.mems_gyro_range * 3.5e-5;
    imu_data.mems_gyro.z = bin_payload.wz1 * imu_data.mems_gyro_range * 3.5e-5;

    // calculate fog gyro data
    imu_data.fog_gyro.x = bin_payload.og_wx * 1e-7;
    imu_data.fog_gyro.y = bin_payload.og_wy * 1e-7;
    imu_data.fog_gyro.z = bin_payload.og_wz * 1e-7;

    // calculate mag data
    imu_data.mag.x = bin_payload.mag_x / 4096;
    imu_data.mag.y = bin_payload.mag_y / 4096;
    imu_data.mag.z = bin_payload.mag_z / 4096;

    // calculate temperature
    imu_data.temp = bin_payload.temp / 100;

    // transfer statuses
    imu_data.fusion_status_x = bin_payload.fusion_status_x;
    imu_data.fusion_status_y = bin_payload.fusion_status_y;
    imu_data.fusion_status_z = bin_payload.fusion_status_z;

}

#endif // AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED 
