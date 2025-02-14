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

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ANELLOX3_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_HAL/AP_HAL.h>


class AP_ExternalAHRS_AnelloX3: public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_AnelloX3(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // check for new data
    void update() override {
        build_packet();
    };
    
    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;
    virtual uint8_t num_gps_sensors(void) const override;

    void build_packet();

    void post_imu() const;

    void update_thread();


private:
    // UART port config
    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;
    
    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;
};

#endif  // AP_EXTERNAL_AHRS_ANELLOX3_ENABLED
