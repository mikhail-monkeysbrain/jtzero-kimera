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

# Temporary build copy only: bind stable camera by-id to current /dev/videoX.
sed -i "s#constexpr const char\* CAMERA_DEVICE=\"/dev/video0\";#constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";#" \
  "$TMP/camera_imu_extrinsics_logger.cpp"

grep -Fq "constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";" "$TMP/camera_imu_extrinsics_logger.cpp" || {
  echo "FATAL: camera substitution verification failed" >&2
  exit 4
}

# Compact full-frame GUI. Telemetry is overlaid on the video, so it also fits
# small VNC/fullscreen desktops. For the first 2 seconds of every new phase a
# large centered instruction is shown over the camera image.
cat > "$TMP/gui_p11.cppfrag" <<'CPP'
void drawGuiP11(cv::Mat& screen,
                const cv::Mat& gray,
                double elapsed,
                double rr,
                double rp,
                double ry,
                bool have_att,
                bool zero_ready) {
  screen.setTo(cv::Scalar(12,12,12));

  if (!gray.empty()) {
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    cv::resize(bgr, bgr, screen.size());
    bgr.copyTo(screen);
  }

  // Dark translucent header/footer directly over video.
  {
    cv::Mat ov = screen.clone();
    cv::rectangle(ov,{0,0,screen.cols,105},{0,0,0},cv::FILLED);
    cv::rectangle(ov,{0,screen.rows-105,screen.cols,105},{0,0,0},cv::FILLED);
    cv::addWeighted(ov,0.72,screen,0.28,0,screen);
  }

  textP11(screen,"JT-ZERO — P11",{25,38},{255,255,255},25);

  if (!zero_ready) {
    textP11(screen,"КАЛИБРОВКА НУЛЯ — НЕ ДВИГАТЬ",{25,82},{0,220,255},28);
    return;
  }

  const auto ph = phaseP11(elapsed);
  const double yaw_err = wrap180(ph.target_yaw - ry);
  const bool rp_ok = std::abs(rr)<=RP_OK_DEG && std::abs(rp)<=RP_OK_DEG;
  const bool yaw_ok = std::abs(yaw_err)<=YAW_OK_DEG;

  std::ostringstream top, timer;
  if (ph.cycle > 0) top << "ЦИКЛ " << ph.cycle << "/3   ";
  top << ph.instruction;
  timer << std::fixed << std::setprecision(1) << std::max(0.0,ph.remain) << " с";
  textP11(screen,top.str(),{25,82},{0,220,255},24);
  textP11(screen,timer.str(),{screen.cols-125,38},{255,255,255},22);

  // Compact telemetry in the footer: nothing can run outside the window.
  std::ostringstream a,b,c;
  a << "ROLL " << std::showpos << std::fixed << std::setprecision(1) << rr
    << "°   PITCH " << rp << "°";
  c << "YAW " << std::showpos << std::fixed << std::setprecision(1) << ry
    << "°   ЦЕЛЬ " << std::setprecision(0) << ph.target_yaw
    << "°   ОШИБКА " << std::setprecision(1) << yaw_err << "°";
  textP11(screen,a.str(),{25,screen.rows-65},
          rp_ok ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255),24);
  textP11(screen,c.str(),{25,screen.rows-25},
          yaw_ok ? cv::Scalar(0,255,0) : cv::Scalar(0,220,255),24);

  if (!have_att)
    textP11(screen,"НЕТ ATTITUDE ОТ FC",{screen.cols-330,screen.rows-65},{0,0,255},24);

  // Determine phase duration from its kind. phase_age resets at every mode switch.
  double phase_duration = PRE_SEC;
  if (ph.name.find("_OUT") != std::string::npos) phase_duration = ROT_SEC;
  else if (ph.name.find("_HOLD") != std::string::npos) phase_duration = HOLD_SEC;
  else if (ph.name.find("_HOME") != std::string::npos) phase_duration = HOME_SEC;
  else if (ph.name == "HOME1" || ph.name == "HOME2") phase_duration = HOME_HOLD_SEC;
  else if (ph.name == "FINAL") phase_duration = FINAL_HOLD_SEC;
  const double phase_age = std::max(0.0,phase_duration-ph.remain);

  // Big mode-change notification for 2 seconds over the video.
  if (phase_age < 2.0) {
    cv::Mat ov = screen.clone();
    const int box_w = std::min(900,screen.cols-80);
    const int box_h = 190;
    const int bx = (screen.cols-box_w)/2;
    const int by = (screen.rows-box_h)/2;
    cv::rectangle(ov,{bx,by,box_w,box_h},{0,0,0},cv::FILLED);
    cv::addWeighted(ov,0.82,screen,0.18,0,screen);

    std::ostringstream mode;
    if (ph.cycle > 0) mode << "ЦИКЛ " << ph.cycle << "/3";
    else mode << "ПОДГОТОВКА";
    textP11(screen,mode.str(),{bx+35,by+55},{255,255,255},30);
    textP11(screen,ph.instruction,{bx+35,by+115},{0,220,255},30);
  }

  // Persistent short action hint after the large notification disappears.
  if (phase_age >= 2.0) {
    std::string hint;
    cv::Scalar hc(0,220,255);
    if (!ph.moving && yaw_ok) {
      hint = "СТОП — ДЕРЖАТЬ";
      hc = cv::Scalar(0,255,0);
    } else if (yaw_err > YAW_OK_DEG) {
      hint = "YAW → +";
    } else if (yaw_err < -YAW_OK_DEG) {
      hint = "YAW → -";
    } else {
      hint = "ЦЕЛЬ ДОСТИГНУТА";
      hc = cv::Scalar(0,255,0);
    }
    textP11(screen,hint,{screen.cols-260,82},hc,24);
  }
}
CPP

# Replace only drawGuiP11() in the temporary C++ build copy.
awk -v frag="$TMP/gui_p11.cppfrag" '
  BEGIN {skip=0}
  /^void drawGuiP11\(/ {
    while ((getline line < frag) > 0) print line;
    close(frag);
    skip=1;
    next;
  }
  skip && /^void stopRatesP11\(/ {skip=0; print; next}
  !skip {print}
' "$TMP/record_p11_final_regression.cpp" > "$TMP/record_p11_final_regression.new.cpp"
mv "$TMP/record_p11_final_regression.new.cpp" "$TMP/record_p11_final_regression.cpp"

grep -Fq "Big mode-change notification" "$TMP/record_p11_final_regression.cpp" || {
  echo "FATAL: compact GUI substitution failed" >&2
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
