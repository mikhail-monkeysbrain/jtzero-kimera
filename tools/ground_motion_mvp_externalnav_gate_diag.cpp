// JT-Zero diagnostic wrapper: программно разрывает только публикацию ExternalNav.
// Production estimator остаётся без изменений: камера, KLT, интеграция x/y,
// ATTITUDE, TF-Luna и DISTANCE_SENSOR продолжают работать.
//
// Если существует файл из JTZERO_GM_EXTNAV_GATE_FILE, вызов send() для
// VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE подавляется целиком.
// DISTANCE_SENSOR никогда не блокируется этим gate.

#include <cstdlib>
#include <unistd.h>
#include "ground_motion_mavlink.hpp"

struct GroundMotionMavlinkPublisherExternalNavGateDiag {
    uint8_t system_id = 191;
    uint8_t component_id = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

    bool gateClosed() const {
        const char* path = std::getenv("JTZERO_GM_EXTNAV_GATE_FILE");
        return path && *path && ::access(path, F_OK) == 0;
    }

    GroundMotionMavlinkPublisher makeInner() const {
        GroundMotionMavlinkPublisher p;
        p.system_id = system_id;
        p.component_id = component_id;
        return p;
    }

    bool send(int fd,
              uint64_t time_usec,
              bool valid,
              double quality01,
              double x_nwu,
              double y_nwu,
              double vx_nwu,
              double vy_nwu) const {
        if (gateClosed()) return false;
        auto p = makeInner();
        return p.send(fd, time_usec, valid, quality01, x_nwu, y_nwu, vx_nwu, vy_nwu);
    }

    bool sendDistanceSensor(int fd,
                            uint32_t time_boot_ms,
                            double distance_m) const {
        auto p = makeInner();
        return p.sendDistanceSensor(fd, time_boot_ms, distance_m);
    }
};

// ground_motion_mvp.cpp повторно включает ground_motion_mavlink.hpp, но там
// #pragma once, поэтому исходный класс уже определён и здесь безопасно
// подменяется только имя типа, используемого production main().
#define GroundMotionMavlinkPublisher GroundMotionMavlinkPublisherExternalNavGateDiag
#include "ground_motion_mvp.cpp"
#undef GroundMotionMavlinkPublisher
