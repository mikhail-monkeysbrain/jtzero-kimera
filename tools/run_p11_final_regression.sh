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

sed -i "s#constexpr const char\* CAMERA_DEVICE=\"/dev/video0\";#constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";#" \
  "$TMP/camera_imu_extrinsics_logger.cpp"

grep -Fq "constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";" "$TMP/camera_imu_extrinsics_logger.cpp" || {
  echo "FATAL: camera substitution verification failed" >&2
  exit 4
}

# P11 operator guidance must not use ArduPilot ATTITUDE/EKF angles.
# Inject a RAW HIGHRES_IMU guide state into the temporary build only:
# - yaw: bias-corrected integral of raw z gyro
# - roll/pitch: gravity-vector tilt from raw accelerometer, relative to ZERO
# ATTITUDE and quaternion remain recorded as diagnostics only.
perl -0777 -i -pe 's@struct AttP11 \{@struct RawGuideP11 {\n  bool initialized = false;\n  int64_t last_ns = 0;\n  double gx_bias = 0, gy_bias = 0, gz_bias = 0;\n  double roll0 = 0, pitch0 = 0;\n  double roll = 0, pitch = 0, yaw = 0;\n};\n\nstruct AttP11 {@s' "$TMP/record_p11_final_regression.cpp"

# Add helper functions before PhaseP11.
perl -0777 -i -pe 's@struct PhaseP11 \{@double rawRollP11(double ax, double ay, double az) {\n  return std::atan2(-ay, -az) * 180.0 / M_PI;\n}\n\ndouble rawPitchP11(double ax, double ay, double az) {\n  return std::atan2(ax, std::sqrt(ay*ay + az*az)) * 180.0 / M_PI;\n}\n\nstruct PhaseP11 {@s' "$TMP/record_p11_final_regression.cpp"

# Declare guide state and ZERO accumulators immediately after AttP11 declaration in main.
perl -0777 -i -pe 's@(AttP11\s+att\s*;)@$1\n    RawGuideP11 rawGuide;\n    double zero_gx_sum=0, zero_gy_sum=0, zero_gz_sum=0;\n    double zero_ax_sum=0, zero_ay_sum=0, zero_az_sum=0;\n    uint64_t zero_imu_n=0;@s' "$TMP/record_p11_final_regression.cpp"

# Feed RAW guide from every HIGHRES_IMU message. This is visualization only.
perl -0777 -i -pe 's@(mavlink_highres_imu_t\s+hi\s*\{\};\s*mavlink_msg_highres_imu_decode\(&mm,&hi\);)@$1\n\n              const int64_t guide_ns = monotonicNs();\n              if (!zero_ready) {\n                zero_gx_sum += hi.xgyro; zero_gy_sum += hi.ygyro; zero_gz_sum += hi.zgyro;\n                zero_ax_sum += hi.xacc;  zero_ay_sum += hi.yacc;  zero_az_sum += hi.zacc;\n                ++zero_imu_n;\n              } else {\n                if (!rawGuide.initialized) { rawGuide.initialized=true; rawGuide.last_ns=guide_ns; }\n                double dt=(guide_ns-rawGuide.last_ns)*1e-9;\n                rawGuide.last_ns=guide_ns;\n                if (dt>0 && dt<0.1) rawGuide.yaw += (hi.zgyro-rawGuide.gz_bias)*dt*180.0/M_PI;\n                const double rr_abs=rawRollP11(hi.xacc,hi.yacc,hi.zacc);\n                const double rp_abs=rawPitchP11(hi.xacc,hi.yacc,hi.zacc);\n                rawGuide.roll = wrap180(rr_abs-rawGuide.roll0);\n                rawGuide.pitch = wrap180(rp_abs-rawGuide.pitch0);\n              }@s or die "FATAL: HIGHRES injection point not found\n"' "$TMP/record_p11_final_regression.cpp"

# At ZERO completion, derive gyro bias and gravity reference from HIGHRES samples.
perl -0777 -i -pe 's@(zero_ready\s*=\s*true\s*;)@if (zero_imu_n < 100) throw std::runtime_error("not enough HIGHRES_IMU samples during ZERO");\n          rawGuide.gx_bias=zero_gx_sum/zero_imu_n; rawGuide.gy_bias=zero_gy_sum/zero_imu_n; rawGuide.gz_bias=zero_gz_sum/zero_imu_n;\n          const double zax=zero_ax_sum/zero_imu_n, zay=zero_ay_sum/zero_imu_n, zaz=zero_az_sum/zero_imu_n;\n          rawGuide.roll0=rawRollP11(zax,zay,zaz); rawGuide.pitch0=rawPitchP11(zax,zay,zaz);\n          rawGuide.yaw=0; rawGuide.last_ns=monotonicNs(); rawGuide.initialized=true;\n          std::cout << "[RAW GUIDE ZERO] gyro_bias=[" << rawGuide.gx_bias << "," << rawGuide.gy_bias << "," << rawGuide.gz_bias << "] gravity_rp=[" << rawGuide.roll0 << "," << rawGuide.pitch0 << "] n=" << zero_imu_n << "\\n";\n          $1@s or die "FATAL: zero_ready injection point not found\n"' "$TMP/record_p11_final_regression.cpp"

# Replace GUI function: compact full-frame RAW HIGHRES guide, no ATTITUDE values.
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
    cv::Mat bgr; cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
    cv::resize(bgr,bgr,screen.size()); bgr.copyTo(screen);
  }
  cv::Mat ov=screen.clone();
  cv::rectangle(ov,{0,0,screen.cols,105},{0,0,0},cv::FILLED);
  cv::rectangle(ov,{0,screen.rows-105,screen.cols,105},{0,0,0},cv::FILLED);
  cv::addWeighted(ov,0.72,screen,0.28,0,screen);
  textP11(screen,"JT-ZERO — P11   GUIDE: RAW HIGHRES IMU",{20,38},{255,255,255},22);
  if (!zero_ready) {
    textP11(screen,"КАЛИБРОВКА RAW IMU — НЕ ДВИГАТЬ",{20,80},{0,220,255},27);
    return;
  }
  const auto ph=phaseP11(elapsed);
  const double yaw_err=ph.target_yaw-ry;
  const bool rp_ok=std::abs(rr)<=RP_OK_DEG && std::abs(rp)<=RP_OK_DEG;
  const bool yaw_ok=std::abs(yaw_err)<=YAW_OK_DEG;
  std::ostringstream t,a,c;
  t<<std::fixed<<std::setprecision(1)<<std::max(0.0,ph.remain)<<" с";
  a<<"RAW R "<<std::showpos<<std::fixed<<std::setprecision(1)<<rr<<"°   P "<<rp<<"°";
  c<<"RAW YAW "<<std::showpos<<std::fixed<<std::setprecision(1)<<ry<<"°   ЦЕЛЬ "<<std::setprecision(0)<<ph.target_yaw<<"°   ERR "<<std::setprecision(1)<<yaw_err<<"°";
  textP11(screen,t.str(),{screen.cols-115,38},{255,255,255},21);
  textP11(screen,a.str(),{20,screen.rows-64},rp_ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),23);
  textP11(screen,c.str(),{20,screen.rows-24},yaw_ok?cv::Scalar(0,255,0):cv::Scalar(0,220,255),23);
  double d=PRE_SEC;
  if(ph.name.find("_OUT")!=std::string::npos)d=ROT_SEC;
  else if(ph.name.find("_HOLD")!=std::string::npos)d=HOLD_SEC;
  else if(ph.name.find("_HOME")!=std::string::npos)d=HOME_SEC;
  else if(ph.name=="HOME1"||ph.name=="HOME2")d=HOME_HOLD_SEC;
  else if(ph.name=="FINAL")d=FINAL_HOLD_SEC;
  const double age=std::max(0.0,d-ph.remain);
  if(age<2.0){
    cv::Mat x=screen.clone(); int bw=std::min(900,screen.cols-60),bh=180,bx=(screen.cols-bw)/2,by=(screen.rows-bh)/2;
    cv::rectangle(x,{bx,by,bw,bh},{0,0,0},cv::FILLED); cv::addWeighted(x,0.84,screen,0.16,0,screen);
    std::ostringstream m; if(ph.cycle>0)m<<"ЦИКЛ "<<ph.cycle<<"/3"; else m<<"ПОДГОТОВКА";
    textP11(screen,m.str(),{bx+30,by+55},{255,255,255},30);
    textP11(screen,ph.instruction,{bx+30,by+115},{0,220,255},28);
  } else {
    std::string h; cv::Scalar hc(0,220,255);
    if(yaw_ok){h="СТОП — ДЕРЖАТЬ";hc={0,255,0};}
    else if(yaw_err>0)h="YAW → +"; else h="YAW → -";
    textP11(screen,h,{screen.cols-250,80},hc,24);
  }
}
CPP

awk -v frag="$TMP/gui_p11.cppfrag" '
  BEGIN {skip=0}
  /^void drawGuiP11\(/ {while((getline line<frag)>0)print line;close(frag);skip=1;next}
  skip && /^void stopRatesP11\(/ {skip=0;print;next}
  !skip {print}
' "$TMP/record_p11_final_regression.cpp" > "$TMP/record_p11_final_regression.new.cpp"
mv "$TMP/record_p11_final_regression.new.cpp" "$TMP/record_p11_final_regression.cpp"

# Route only GUI arguments to RAW guide. ATTITUDE continues to be logged unchanged.
perl -0777 -i -pe 's@(drawGuiP11\(\s*screen,\s*last_gray,\s*elapsed,\s*)rr,\s*rp,\s*ry,@$1rawGuide.roll, rawGuide.pitch, rawGuide.yaw,@s or die "FATAL: drawGui call not found\n"' "$TMP/record_p11_final_regression.cpp"

grep -Fq "GUIDE: RAW HIGHRES IMU" "$TMP/record_p11_final_regression.cpp" || { echo "FATAL: RAW GUI substitution failed" >&2; exit 5; }
grep -Fq "[RAW GUIDE ZERO]" "$TMP/record_p11_final_regression.cpp" || { echo "FATAL: RAW ZERO injection failed" >&2; exit 6; }

echo "[BUILD] $BIN"
g++ -std=c++17 -O2 \
  "$TMP/record_p11_final_regression.cpp" -o "$BIN" \
  -I"$TMP" -I"$ROOT/tools" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -lpthread $(pkg-config --libs opencv4)

echo "[RUN] P11 final regression — RAW guide"
LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN"
