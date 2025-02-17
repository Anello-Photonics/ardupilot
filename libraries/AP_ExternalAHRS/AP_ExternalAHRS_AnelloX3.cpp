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

#if AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED

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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "update_thread run");
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
    }
}



// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_AnelloX3::post_imu() const
{
    // dummy function for now
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
    // dummy function for now
    return true;
}

bool AP_ExternalAHRS_AnelloX3::initialised(void) const
{
    // dummy function for now
    return true;
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

#endif // AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED 
