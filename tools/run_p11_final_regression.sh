#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
CAM_BY_ID=/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0
TMP=/tmp/jtzero_p11_final_regression
BIN=/tmp/record_p11_final_regression

if [[ ! -e "$CAM_BY_ID" ]]; then
  echo "FATAL: OV9281 camera by-id not found: $CAM_BY_ID" >&2
  ls -l /dev/v4l/by-id/ 2>/dev/null || true
  exit 2
fi

REAL_CAM="$(readlink -f "$CAM_BY_ID")"
if [[ ! -c "$REAL_CAM" ]]; then
  echo "FATAL: resolved camera is not a video character device: $REAL_CAM" >&2
  exit 3
fi

echo "[CAM] $CAM_BY_ID -> $REAL_CAM"
echo "[MAV] expected serial: /dev/ttyAMA0 @ 460800"

rm -rf "$TMP"
mkdir -p "$TMP"
cp "$ROOT/tools/camera_imu_extrinsics_logger.cpp" "$TMP/"
cp "$ROOT/tools/camera_imu_timestamp_policy.hpp" "$TMP/"
cp "$ROOT/tools/record_p11_final_regression.cpp" "$TMP/"

# Temporary build copy only: bind the stable camera by-id to its current /dev/videoX.
sed -i "s#constexpr const char\* CAMERA_DEVICE=\"/dev/video0\";#constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";#" \
  "$TMP/camera_imu_extrinsics_logger.cpp"

grep -Fq "constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";" "$TMP/camera_imu_extrinsics_logger.cpp" || {
  echo "FATAL: camera substitution verification failed" >&2
  exit 4
}

# Replace the three moving gauges in the temporary build with deterministic numeric guidance.
# The recorded data path is untouched; this changes only operator visualization.
perl -0777 -i -pe 's@    drawGaugeP11\(\n        screen,840,185,380,\n        "ROLL",rr,0,RP_OK_DEG,-20,20\);\n\n    drawGaugeP11\(\n        screen,840,285,380,\n        "PITCH",rp,0,RP_OK_DEG,-20,20\);\n\n    drawGaugeP11\(\n        screen,840,385,380,\n        "YAW",ry,ph\.target_yaw,YAW_OK_DEG,-120,120\);@    {\n      std::ostringstream s1, s2, s3, s4;\n      const double yaw_err = wrap180(ph.target_yaw - ry);\n      s1 << "ROLL:  " << std::fixed << std::setprecision(1) << rr << "°   (норма ±" << RP_OK_DEG << "°)";\n      s2 << "PITCH: " << std::fixed << std::setprecision(1) << rp << "°   (норма ±" << RP_OK_DEG << "°)";\n      s3 << "YAW СЕЙЧАС: " << std::showpos << std::fixed << std::setprecision(1) << ry << "°";\n      s4 << "ЦЕЛЬ: " << std::showpos << std::fixed << std::setprecision(0) << ph.target_yaw\n         << "°    ОШИБКА: " << std::setprecision(1) << yaw_err << "°";\n      textP11(screen,s1.str(),{840,185},std::abs(rr)<=RP_OK_DEG ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255),24);\n      textP11(screen,s2.str(),{840,235},std::abs(rp)<=RP_OK_DEG ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255),24);\n      textP11(screen,s3.str(),{840,315},cv::Scalar(255,255,255),30);\n      textP11(screen,s4.str(),{840,365},std::abs(yaw_err)<=YAW_OK_DEG ? cv::Scalar(0,255,0) : cv::Scalar(0,220,255),24);\n\n      if (std::abs(yaw_err) <= YAW_OK_DEG) {\n        textP11(screen,"СТОП — ДЕРЖАТЬ",{840,420},cv::Scalar(0,255,0),30);\n      } else if (yaw_err > 0) {\n        textP11(screen,"ПОВОРАЧИВАЙ YAW В СТОРОНУ +",{840,420},cv::Scalar(0,220,255),24);\n      } else {\n        textP11(screen,"ПОВОРАЧИВАЙ YAW В СТОРОНУ -",{840,420},cv::Scalar(0,220,255),24);\n      }\n    }@s or die "FATAL: GUI substitution failed\n"' \
  "$TMP/record_p11_final_regression.cpp"

grep -Fq "YAW СЕЙЧАС" "$TMP/record_p11_final_regression.cpp" || {
  echo "FATAL: deterministic GUI substitution verification failed" >&2
  exit 5
}

echo "[BUILD] $BIN"
g++ -std=c++17 -O2 \
  "$TMP/record_p11_final_regression.cpp" \
  -o "$BIN" \
  -I"$TMP" \
  -I"$ROOT/tools" \
  -I"$K/third_party/mavlink" \
  -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  -lpthread \
  $(pkg-config --libs opencv4)

echo "[RUN] P11 final regression"
LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN"
