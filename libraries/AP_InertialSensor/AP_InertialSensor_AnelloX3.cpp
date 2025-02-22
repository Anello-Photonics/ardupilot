/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_AnelloX3.h"

extern const AP_HAL::HAL& hal;

AP_InertialSensor_AnelloX3::AP_InertialSensor_AnelloX3(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_AnelloX3::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation)
                                   
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_AnelloX3(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_AnelloX3::start()
{
    // replace this with how we register a new serial device
    //if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1647X)) ||
    //    !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1647X))) {
    //    return;
    //}

    // wait on this
    //set_gyro_orientation(gyro_instance, rotation);
    //set_accel_orientation(accel_instance, rotation);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_AnelloX3::loop, void),
                                      "AnelloX3",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create AnelloX3 thread");
    }
}

bool AP_InertialSensor_AnelloX3::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    // will this be relevant to us?
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    return true;
}

/*
  sensor read loop
 */
void AP_InertialSensor_AnelloX3::loop(void)
{
    while (true) {
        // code here for reading what comes in over the serial port
        }
    }
}

bool AP_InertialSensor_AnelloX3::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
