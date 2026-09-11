#pragma once
// MAVLink publisher для Ground Motion MVP.
// Публикует:
//   1) VISION_POSITION_ESTIMATE — относительная горизонтальная позиция ExternalNav;
//   2) VISION_SPEED_ESTIMATE — горизонтальная скорость ExternalNav;
//   3) DISTANCE_SENSOR — TF-Luna для AP_RangeFinder MAVLink backend.
// Сам по себе НЕ переключает EKF и НЕ меняет параметры FC.
//
// Position и velocity происходят из одной Ground Motion оценки и потому
// коррелированы. Это не два независимых датчика, а единый ExternalNav output.
// Yaw estimator не публикует как источник EKF yaw.
//
// Вход estimator: NWU. Для ArduPilot преобразование в NED:
// X остаётся, Y меняет знак. Z position намеренно 0: вертикальная позиция
// поступает в EKF отдельно от TF-Luna через DISTANCE_SENSOR/POSZ=RangeFinder.

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <unistd.h>
#include "common/mavlink.h"

struct GroundMotionMavlinkPublisher {
    uint8_t system_id = 191;
    uint8_t component_id = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

    static bool writeMessage(int fd, const mavlink_message_t& msg) {
        if (fd < 0) return false;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        size_t off = 0;
        while (off < n) {
            const ssize_t k = ::write(fd, buf + off, n - off);
            if (k > 0) { off += static_cast<size_t>(k); continue; }
            if (k < 0 && errno == EINTR) continue;
            if (k < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
            return false;
        }
        return true;
    }

    bool sendPosition(int fd,
                      uint64_t time_usec,
                      bool valid,
                      double x_nwu,
                      double y_nwu) const {
        if (fd < 0 || !valid) return false;
        if (!std::isfinite(x_nwu) || !std::isfinite(y_nwu)) return false;

        const float x_ned = static_cast<float>(x_nwu);
        const float y_ned = static_cast<float>(-y_nwu);

        // ВАЖНО: ArduPilot для VISION_POSITION_ESTIMATE сворачивает
        // covariance X/Y/Z в один сферический posErr:
        // sqrt(cov_x + cov_y + cov_z). Поэтому нельзя ставить огромную
        // дисперсию Z для обозначения "Z не измеряется" — это одновременно
        // делает XY position практически бесполезной для EKF.
        //
        // POSZ у нас выбран RangeFinder, поэтому ExternalNav Z не является
        // источником вертикальной позиции. Здесь задаём суммарный posErr=0.20 m,
        // согласованный с VISO_POS_M_NSE=0.20:
        // sqrt(3 * 0.013333333) ~= 0.20 m.
        constexpr float pose_axis_var = 0.013333333f;
        float covariance[21]{};
        covariance[0] = pose_axis_var;
        covariance[6] = pose_axis_var;
        covariance[11] = pose_axis_var;

        // Attitude не используется как EKF yaw source (EK3_SRC1_YAW=None),
        // но ArduPilot также сворачивает R/P/Y covariance в один angErr.
        // Оставляем конечные консервативные значения вместо прежних 100 rad^2.
        constexpr float attitude_axis_var = 0.25f;
        covariance[15] = attitude_axis_var;
        covariance[18] = attitude_axis_var;
        covariance[20] = attitude_axis_var;

        mavlink_message_t msg{};
        mavlink_msg_vision_position_estimate_pack(
            system_id,
            component_id,
            &msg,
            time_usec,
            x_ned,
            y_ned,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            covariance,
            0);
        return writeMessage(fd, msg);
    }

    bool sendVelocity(int fd,
                      uint64_t time_usec,
                      bool valid,
                      double quality01,
                      double vx_nwu,
                      double vy_nwu) const {
        (void)quality01;
        if (fd < 0 || !valid) return false;
        if (!std::isfinite(vx_nwu) || !std::isfinite(vy_nwu)) return false;

        const float vx_ned = static_cast<float>(vx_nwu);
        const float vy_ned = static_cast<float>(-vy_nwu);
        constexpr float var_v = 0.01f; // sigma=0.10 m/s
        float covariance[9] = {var_v,0,0, 0,var_v,0, 0,0,var_v};

        mavlink_message_t msg{};
        mavlink_msg_vision_speed_estimate_pack(
            system_id, component_id, &msg, time_usec,
            vx_ned, vy_ned, 0.0f, covariance, 0);
        return writeMessage(fd, msg);
    }

    // Сохраняем прежний production API. Один вызов отправляет согласованную пару
    // position + velocity из одной оценки одного кадра.
    bool send(int fd,
              uint64_t time_usec,
              bool valid,
              double quality01,
              double x_nwu,
              double y_nwu,
              double vx_nwu,
              double vy_nwu) const {
        if (fd < 0 || !valid) return false;
        const bool pos_ok = sendPosition(fd, time_usec, valid, x_nwu, y_nwu);
        const bool vel_ok = sendVelocity(fd, time_usec, valid, quality01, vx_nwu, vy_nwu);
        return pos_ok && vel_ok;
    }

    bool sendDistanceSensor(int fd,
                            uint32_t time_boot_ms,
                            double distance_m) const {
        if (fd < 0 || !std::isfinite(distance_m)) return false;
        if (distance_m < 0.10 || distance_m > 8.00) return false;
        const uint16_t current_cm = static_cast<uint16_t>(
            std::lround(std::clamp(distance_m, 0.10, 8.00) * 100.0));
        constexpr uint16_t min_cm = 10;
        constexpr uint16_t max_cm = 800;
        constexpr uint8_t sensor_id = 0;
        constexpr uint8_t covariance = 0;
        float quaternion[4] = {0,0,0,0};
        mavlink_message_t msg{};
        mavlink_msg_distance_sensor_pack(
            system_id, component_id, &msg, time_boot_ms,
            min_cm, max_cm, current_cm, MAV_DISTANCE_SENSOR_LASER,
            sensor_id, MAV_SENSOR_ROTATION_PITCH_270, covariance,
            0.0f, 0.0f, quaternion, 0);
        return writeMessage(fd, msg);
    }
};
