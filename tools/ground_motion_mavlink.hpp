#pragma once
// MAVLink publisher для Ground Motion MVP.
// Отправляет ODOMETRY в ArduPilot. Сам по себе НЕ переключает EKF и НЕ меняет параметры FC.
// Координаты estimator: NWU. MAVLink LOCAL_FRD: Y и Z имеют противоположный знак.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unistd.h>
#include "common/mavlink.h"

struct GroundMotionMavlinkPublisher {
    uint8_t system_id = 42;
    uint8_t component_id = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    uint8_t reset_counter = 0;

    bool send(int fd,
              uint64_t time_usec,
              bool valid,
              double quality01,
              double x_nwu,
              double y_nwu,
              double vx_nwu,
              double vy_nwu) const {
        if (fd < 0 || !valid) return false; // fail-closed: плохие оценки не публикуем

        const float x_frd = static_cast<float>(x_nwu);
        const float y_frd = static_cast<float>(-y_nwu);
        const float z_frd = 0.0f;
        const float vx_frd = static_cast<float>(vx_nwu);
        const float vy_frd = static_cast<float>(-vy_nwu);
        const float vz_frd = 0.0f;
        const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

        float pose_cov[21];
        float vel_cov[21];
        for (float &v : pose_cov) v = NAN;
        for (float &v : vel_cov) v = NAN;

        const int quality_pct = std::clamp(static_cast<int>(std::lround(quality01 * 100.0)), 1, 100);

        mavlink_message_t msg{};
        mavlink_msg_odometry_pack(
            system_id, component_id, &msg,
            time_usec,
            MAV_FRAME_LOCAL_FRD,
            MAV_FRAME_LOCAL_FRD,
            x_frd, y_frd, z_frd,
            q,
            vx_frd, vy_frd, vz_frd,
            0.0f, 0.0f, 0.0f,
            pose_cov, vel_cov,
            reset_counter,
            MAV_ESTIMATOR_TYPE_VIO,
            static_cast<int8_t>(quality_pct));

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        size_t off = 0;
        while (off < n) {
            const ssize_t k = ::write(fd, buf + off, n - off);
            if (k > 0) { off += static_cast<size_t>(k); continue; }
            if (k < 0 && errno == EINTR) continue;
            // Не блокируем vision loop из-за забитого serial TX.
            if (k < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
            return false;
        }
        return true;
    }
};
