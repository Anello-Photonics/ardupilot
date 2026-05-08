#include "LR_MsgHandler.h"
#include "LogReader.h"
#include "Replay.h"

#include <AP_DAL/AP_DAL.h>

#include <cinttypes>
#include <fstream>

#include <string>
#include <sstream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <map>

extern const AP_HAL::HAL& hal;

#define MSG_CREATE(sname,msgbytes) log_ ##sname msg; memcpy((void*)&msg, (msgbytes)+3, sizeof(msg));

LR_MsgHandler::LR_MsgHandler(struct log_Format &_f) :
    MsgHandler(_f) {
}

void LR_MsgHandler_RFRH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RFRF::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRF, msgbytes);
#define MAP_FLAG(flag1, flag2) if (msg.frame_types & uint8_t(flag1)) msg.frame_types |= uint8_t(flag2)
    /*
      when we force an EKF we map the trigger flags over
     */
    if (replay_force_ekf2) {
        MAP_FLAG(AP_DAL::FrameType::InitialiseFilterEKF3, AP_DAL::FrameType::InitialiseFilterEKF2);
        MAP_FLAG(AP_DAL::FrameType::UpdateFilterEKF3, AP_DAL::FrameType::UpdateFilterEKF2);
        MAP_FLAG(AP_DAL::FrameType::LogWriteEKF3, AP_DAL::FrameType::LogWriteEKF2);
    }
    if (replay_force_ekf3) {
        MAP_FLAG(AP_DAL::FrameType::InitialiseFilterEKF2, AP_DAL::FrameType::InitialiseFilterEKF3);
        MAP_FLAG(AP_DAL::FrameType::UpdateFilterEKF2, AP_DAL::FrameType::UpdateFilterEKF3);
        MAP_FLAG(AP_DAL::FrameType::LogWriteEKF2, AP_DAL::FrameType::LogWriteEKF3);
    }
#undef MAP_FLAG
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_RFRN::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRN, msgbytes);
    // AP::dal().handle_message(msg);
}

void LR_MsgHandler_REV2::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REV2, msgbytes);

    switch ((AP_DAL::Event)msg.event) {

    case AP_DAL::Event::resetGyroBias:
        ekf2.resetGyroBias();
        break;
    case AP_DAL::Event::resetHeightDatum:
        ekf2.resetHeightDatum();
        break;
    case AP_DAL::Event::setTerrainHgtStable:
        ekf2.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event::unsetTerrainHgtStable:
        ekf2.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event::requestYawReset:
        ekf2.requestYawReset();
        break;
    case AP_DAL::Event::checkLaneSwitch:
        ekf2.checkLaneSwitch();
        break;
    case AP_DAL::Event::setSourceSet0 ... AP_DAL::Event::setSourceSet2:
        break;
    }
    if (replay_force_ekf3) {
        LR_MsgHandler_REV3 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RSO2::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSO2, msgbytes);
    Location loc;
    loc.lat = msg.lat;
    loc.lng = msg.lng;
    loc.alt = msg.alt;
    ekf2.setOriginLLH(loc);

    if (replay_force_ekf3) {
        LR_MsgHandler_RSO2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RWA2::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RWA2, msgbytes);
    ekf2.writeDefaultAirSpeed(msg.airspeed);
    if (replay_force_ekf3) {
        LR_MsgHandler_RWA2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}


void LR_MsgHandler_REV3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REV3, msgbytes);

    switch ((AP_DAL::Event)msg.event) {

    case AP_DAL::Event::resetGyroBias:
        ekf3.resetGyroBias();
        break;
    case AP_DAL::Event::resetHeightDatum:
        ekf3.resetHeightDatum();
        break;
    case AP_DAL::Event::setTerrainHgtStable:
        ekf3.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event::unsetTerrainHgtStable:
        ekf3.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event::requestYawReset:
        ekf3.requestYawReset();
        break;
    case AP_DAL::Event::checkLaneSwitch:
        ekf3.checkLaneSwitch();
        break;
    case AP_DAL::Event::setSourceSet0 ... AP_DAL::Event::setSourceSet2:
        ekf3.setPosVelYawSourceSet(uint8_t(msg.event)-uint8_t(AP_DAL::Event::setSourceSet0));
        break;
    }

    if (replay_force_ekf2) {
        LR_MsgHandler_REV2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RSO3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSO3, msgbytes);
    Location loc;
    loc.lat = msg.lat;
    loc.lng = msg.lng;
    loc.alt = msg.alt;
    ekf3.setOriginLLH(loc);
    if (replay_force_ekf2) {
        LR_MsgHandler_RSO2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RWA3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RWA3, msgbytes);
    ekf3.writeDefaultAirSpeed(msg.airspeed, msg.uncertainty);
    if (replay_force_ekf2) {
        LR_MsgHandler_RWA2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_REY3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REY3, msgbytes);
    ekf3.writeEulerYawAngle(msg.yawangle, msg.yawangleerr, msg.timestamp_ms, msg.type);
}

void LR_MsgHandler_RISH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RISH, msgbytes);
    AP::dal().handle_message(msg);
}
void LR_MsgHandler_RISI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RISI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RASH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RASH, msgbytes);
    // AP::dal().handle_message(msg);
}
void LR_MsgHandler_RASI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RASI, msgbytes);
    // AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBRH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBRH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBRI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBRI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RRNH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RRNH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RRNI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RRNI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPI, msgbytes);
    msg.gps_yaw_deg_returncode = 1;
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPJ::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPJ, msgbytes);
    
    // only read file once
    if (_rgpj_data.empty()) {

        std::string csv_path = "/path/to/merged_data.csv";
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Reading csv yaw file: %s", csv_path.c_str());

        // read in csv and print time value
        std::ifstream file(csv_path.c_str());
        if (!file.is_open()) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Could not open csv yaw file: %s", csv_path.c_str());
            // break;
        }
        std::string line;
        bool first_line = true;

        while (std::getline(file, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            // line: timestamp,TS,VX,VY,VZ,SA,Y,YA,YT,Lat,Lon,Alt,HA,VA,HD,I,As,E2T
            std::istringstream ss(line);
            std::string timestamp_str, ts_str, vx_str, vy_str, vz_str, sa_str;
            std::string y_str, ya_str, yt_str, lat_str, lon_str, alt_str;
            std::string ha_str, va_str, hd_str, i_str, as_str, et_str;

            if (!std::getline(ss, timestamp_str, ',') ||
                !std::getline(ss, ts_str, ',') ||
                !std::getline(ss, vx_str, ',') ||
                !std::getline(ss, vy_str, ',') ||
                !std::getline(ss, vz_str, ',') ||
                !std::getline(ss, sa_str, ',') ||
                !std::getline(ss, y_str, ',') ||
                !std::getline(ss, ya_str, ',') ||
                !std::getline(ss, yt_str, ',') ||
                !std::getline(ss, lat_str, ',') ||
                !std::getline(ss, lon_str, ',') ||
                !std::getline(ss, alt_str, ',') ||
                !std::getline(ss, ha_str, ',') ||
                !std::getline(ss, va_str, ',') ||
                !std::getline(ss, hd_str, ',') ||
                !std::getline(ss, i_str, ',') ||
                !std::getline(ss, as_str, ',') ||
                !std::getline(ss, et_str)) {

                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                    "Malformed line in csv yaw file: %s", line.c_str());
                continue;
            }

            {
                RGPJEntry e;
                e.timestamp = std::stod(timestamp_str);
                e.yaw = std::stod(y_str);
                e.yaw_accuracy = std::stod(ya_str);
                e.yaw_deg_time_ms = std::stoul(yt_str);

                _rgpj_data.push_back(e);

                // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                //     "RGPJ: timestamp=%.2f y=%.2f ya=%.2f",
                //     e.timestamp, e.yaw, e.yaw_accuracy);

            }

            {
                RASIEntry e;
                e.timestamp = std::stod(timestamp_str);
                e.airspeed = std::stod(as_str);
                e.last_update_ms = std::stoul(ts_str);
                _rasi_data.push_back(e);

                // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                //     "RASI: timestamp=%.2f airspeed=%.2f",
                //     e.timestamp, e.airspeed);
            }

            {
                RFRNEntry e;
                e.timestamp = std::stod(timestamp_str);
                e.E2T = std::stod(et_str);
                e.last_update_ms = std::stoul(ts_str);
                _rfrn_data.push_back(e);

                // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                //     "RFRN: timestamp=%.2f E2T=%.2f",
                //     e.timestamp, e.E2T);
            }

            // save off all values from csv for use in replay

        }
        std::sort(_rgpj_data.begin(), _rgpj_data.end(),
            [](const RGPJEntry &a, const RGPJEntry &b) {
                return a.timestamp < b.timestamp;
            });
    }

    // for each new msg, overwrite the yaw and yaw accuracy values with the same index
    // match the time_last_message_time_ms to the csv timestamp and use the closest one
    uint32_t t_anello = msg.last_message_time_ms;
    
    // find closest yaw_deg_time_ms in _rgpj_data to t_anello
    auto it = std::lower_bound(_rgpj_data.begin(), _rgpj_data.end(), t_anello,
        [](const RGPJEntry &e, uint32_t value) {
            return e.yaw_deg_time_ms < value;
        });
    const RGPJEntry *best = nullptr;
    if (it == _rgpj_data.begin()) {
        best = &(*it);
    } else if (it == _rgpj_data.end()) {
        best = &(_rgpj_data.back());
    } else {
        const RGPJEntry *before = &(*(it-1));
        const RGPJEntry *after = &(*it);
        if (std::abs(int64_t(before->yaw_deg_time_ms) - int64_t(t_anello)) < std::abs(int64_t(after->yaw_deg_time_ms) - int64_t(t_anello))) {
            best = before;
        } else {
            best = after;
        }
    }

    if (best) {
        msg.yaw_deg = best->yaw;
        msg.yaw_accuracy_deg = best->yaw_accuracy;
        msg.yaw_deg_time_ms = best->yaw_deg_time_ms;
    }

    AP::dal().handle_message(msg);

    {
        log_RASI rasi;
        memcpy(&rasi, msgbytes+3, sizeof(rasi));

        log_RASH rash;
        memcpy(&rash, msgbytes+3, sizeof(rash));

        log_RFRN rfrn;
        memcpy(&rfrn, msgbytes+3, sizeof(rfrn));

        // find msg for airspeed
        // MSG_CREATE(RASI, msg);
        auto it_as = std::lower_bound(_rasi_data.begin(), _rasi_data.end(), t_anello,
            [](const RASIEntry &e, uint32_t value) {
                return e.last_update_ms < value;
            });
        const RASIEntry *best_as = nullptr;
        if (it_as == _rasi_data.begin()) {
            best_as = &(*it_as);
        } else if (it_as == _rasi_data.end()) {
            best_as = &(_rasi_data.back());
        } else {
            const RASIEntry *before = &(*(it_as-1));
            const RASIEntry *after = &(*it_as);
            if (std::abs(int64_t(before->last_update_ms) - int64_t(t_anello)) < std::abs(int64_t(after->last_update_ms) - int64_t(t_anello))) {
                best_as = before;
            } else {
                best_as = after;    
            }
        }

        if (best_as) {
            rasi.airspeed = best_as->airspeed;
            rasi.last_update_ms = best_as->last_update_ms;
            rasi.healthy = 1;
            rasi.use = 1;
            rasi.instance = 0;

            rash.num_sensors = 1;
            rash.primary = 0;
        }
        AP::dal().handle_message(rasi);
        AP::dal().handle_message(rash);


        // find msg for E2T
        // MSG_CREATE(RFRN, msg);
        auto it_e2t = std::lower_bound(_rfrn_data.begin(), _rfrn_data.end(), t_anello,
            [](const RFRNEntry &e, uint32_t value) {
                return e.last_update_ms < value;
            });
        const RFRNEntry *best_e2t = nullptr;
        if (it_e2t == _rfrn_data.begin()) {
            best_e2t = &(*it_e2t);
        } else if (it_e2t == _rfrn_data.end()) {
            best_e2t = &(_rfrn_data.back());
        } else {
            const RFRNEntry *before = &(*(it_e2t-1));
            const RFRNEntry *after = &(*it_e2t);
            if (std::abs(int64_t(before->last_update_ms) - int64_t(t_anello)) < std::abs(int64_t(after->last_update_ms) - int64_t(t_anello))) {
                best_e2t = before;
            } else {
                best_e2t = after;
            }
        }

        if (best_e2t) {
            // we want to log the E2T value at the same time as the yaw update, so we log it with the same timestamp as the RGPJ message
            rfrn.EAS2TAS = best_e2t->E2T;
            rfrn.available_memory = 316688;
            rfrn.ahrs_airspeed_sensor_enabled = 1;
            rfrn.vehicle_class = uint8_t(AP_DAL::VehicleClass::FIXED_WING);
            rfrn.fly_forward = 1;
            rfrn.armed = 1;
        }
        AP::dal().handle_message(rfrn);

        // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "RGPJ: t_anello=%u, airspeed=%.2f, E2T=%.2f, yaw_deg=%.2f, yaw_accuracy_deg=%.2f, yaw_deg_time_ms=%u",
        //     t_anello, rasi.airspeed, rfrn.EAS2TAS, msg.yaw_deg, msg.yaw_accuracy_deg, msg.yaw_deg_time_ms);
    }
    
}

void LR_MsgHandler_RMGH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RMGH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RMGI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RMGI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBCH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBCH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBCI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBCI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RVOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RVOH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_ROFH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(ROFH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_RWOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RWOH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_RBOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBOH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_REPH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REPH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_REVH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REVH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

#include <AP_AHRS/AP_AHRS.h>
#include "VehicleType.h"

bool LR_MsgHandler_PARM::set_parameter(const char *name, const float value)
{
    const char *ignore_parms[] = {
        "LOG_FILE_BUFSIZE",
        "LOG_DISARMED"
    };
    for (uint8_t i=0; i < ARRAY_SIZE(ignore_parms); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }

    return LogReader::set_parameter(name, value);
}

void LR_MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];

    require_field(msg, "Name", parameter_name, parameter_name_len);

    float value = require_field_float(msg, "Value");
    set_parameter(parameter_name, value);
}
