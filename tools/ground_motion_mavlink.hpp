#pragma once
// MAVLink publisher для Ground Motion MVP.
// Отправляет только VISION_SPEED_ESTIMATE в ArduPilot.
// Сам по себе НЕ переключает EKF и НЕ меняет параметры FC.
//
// Контракт production MVP: Ground Motion является только источником
// горизонтальной скорости ExternalNav. Позицию и yaw estimator не публикует.
//
// Вход estimator: NWU. AP_VisualOdom_MAV::handle_vision_speed_estimate()
// ожидает velocity в NED, поэтому преобразование: X остаётся, Y меняет знак,
// Z меняет знак. Сейчас вертикальная скорость не оценивается и передаётся 0.

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <unistd.h>
#include "common/mavlink.h"

struct GroundMotionMavlinkPublisher {
    uint8_t system_id = 42;
    uint8_t component_id = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

    bool send(int fd,
              uint64_t time_usec,
              bool valid,
              double quality01,
              double x_nwu,
              double y_nwu,
              double vx_nwu,
              double vy_nwu) const {
        (void)quality01; // VISION_SPEED_ESTIMATE не имеет quality поля.
        (void)x_nwu;
        (void)y_nwu;

        // Fail-closed: при невалидном Ground Motion ничего в EKF не отправляем.
        if (fd < 0 || !valid) return false;
        if (!std::isfinite(vx_nwu) || !std::isfinite(vy_nwu)) return false;

        const float vx_ned = static_cast<float>(vx_nwu);
        const float vy_ned = static_cast<float>(-vy_nwu);
        const float vz_ned = 0.0f;

        // Диагональная covariance скорости. AP_VisualOdom извлекает из неё
        // vel_err. Берём консервативные 0.10 m/s по каждой оси для MVP;
        // нижнюю границу всё равно задаёт VISO_VEL_M_NSE на FC.
        constexpr float sigma_v = 0.10f;
        constexpr float var_v = sigma_v * sigma_v;
        float covariance[9] = {
            var_v, 0.0f, 0.0f,
            0.0f, var_v, 0.0f,
            0.0f, 0.0f, var_v
        };

        mavlink_message_t msg{};
        mavlink_msg_vision_speed_estimate_pack(
            system_id,
            component_id,
            &msg,
            time_usec,
            vx_ned,
            vy_ned,
            vz_ned,
            covariance,
            0);

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        size_t off = 0;
        while (off < n) {
            const ssize_t k = ::write(fd, buf + off, n - off);
            if (k > 0) {
                off += static_cast<size_t>(k);
                continue;
            }
            if (k < 0 && errno == EINTR) continue;
            if (k < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
            return false;
        }
        return true;
    }
};
