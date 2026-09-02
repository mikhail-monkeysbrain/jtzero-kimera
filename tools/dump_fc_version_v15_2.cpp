// JT-ZERO v15.2: read-only FC firmware version diagnostic via MAVLink.
// Requests AUTOPILOT_VERSION directly from the flight controller.

#define main jtzero_camera_imu_logger_unused_main_v15_2
#include "camera_imu_extrinsics_logger.cpp"
#undef main

int main() {
  int fd = -1;
  try {
    fd = openSerial();
    std::cout << "[MAV] waiting for HEARTBEAT...\n";

    mavlink_status_t st{};
    mavlink_message_t msg{};
    uint8_t target_sys = 0, target_comp = 0;
    const int64_t deadline = monotonicNs() + 10000000000LL;

    while (monotonicNs() < deadline && target_sys == 0) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t b[2048];
      const ssize_t n = read(fd, b, sizeof(b));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &st) &&
            msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          target_sys = msg.sysid;
          target_comp = msg.compid;
          break;
        }
      }
    }
    if (!target_sys) throw std::runtime_error("HEARTBEAT timeout");

    std::cout << "[MAV] FC sysid=" << int(target_sys)
              << " compid=" << int(target_comp) << "\n";

    mavlink_message_t req{};
    mavlink_msg_command_long_pack(
        255, 190, &req,
        target_sys, target_comp,
        MAV_CMD_REQUEST_MESSAGE,
        0,
        MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0);

    uint8_t out[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(out, &req);
    if (write(fd, out, len) != len) throw std::runtime_error("write request failed");

    bool got = false;
    mavlink_autopilot_version_t v{};
    const int64_t version_deadline = monotonicNs() + 5000000000LL;
    while (monotonicNs() < version_deadline && !got) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t b[4096];
      const ssize_t n = read(fd, b, sizeof(b));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &st)) continue;
        if (msg.msgid == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
          mavlink_msg_autopilot_version_decode(&msg, &v);
          got = true;
          break;
        }
      }
    }

    if (!got) throw std::runtime_error("AUTOPILOT_VERSION timeout");

    auto major = [](uint32_t x) { return (x >> 24) & 0xff; };
    auto minor = [](uint32_t x) { return (x >> 16) & 0xff; };
    auto patch = [](uint32_t x) { return (x >> 8) & 0xff; };
    auto type  = [](uint32_t x) { return x & 0xff; };

    std::cout << "\n================ FC VERSION V15.2 ================\n";
    std::cout << "flight_sw_version raw: " << v.flight_sw_version << "\n";
    std::cout << "flight_sw_version: "
              << major(v.flight_sw_version) << "."
              << minor(v.flight_sw_version) << "."
              << patch(v.flight_sw_version)
              << " type=" << type(v.flight_sw_version) << "\n";
    std::cout << "middleware_sw_version raw: " << v.middleware_sw_version << "\n";
    std::cout << "os_sw_version raw: " << v.os_sw_version << "\n";
    std::cout << "board_version: " << v.board_version << "\n";
    std::cout << "vendor_id: " << v.vendor_id << "\n";
    std::cout << "product_id: " << v.product_id << "\n";
    std::cout << "capabilities: " << v.capabilities << "\n";

    std::cout << "flight_custom_version: ";
    for (size_t i = 0; i < sizeof(v.flight_custom_version); ++i)
      std::cout << std::hex << std::setw(2) << std::setfill('0')
                << int(v.flight_custom_version[i]);
    std::cout << std::dec << "\n";

    std::cout << "READ-ONLY RESULT: PASS\n";
    close(fd);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    if (fd >= 0) close(fd);
    return 1;
  }
}
