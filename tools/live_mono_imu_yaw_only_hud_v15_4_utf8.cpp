// JT-ZERO v15.4 UTF-8 wrapper for live_mono_imu_yaw_only_hud_v15_3.cpp.
//
// Purpose: keep the validated v15.3 IMU/Kimera logic unchanged while replacing
// OpenCV Hershey text rendering with FreeType, so Russian UTF-8 labels render
// correctly in the GUI.
//
// Build must link -lopencv_freetype.

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/freetype.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

namespace cv {
namespace jtzero_utf8_v154 {

inline cv::Ptr<cv::freetype::FreeType2>& renderer() {
  static cv::Ptr<cv::freetype::FreeType2> ft;
  static std::once_flag once;
  std::call_once(once, []() {
    ft = cv::freetype::createFreeType2();

    const char* fonts[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    };

    bool loaded = false;
    for (const char* font : fonts) {
      try {
        ft->loadFontData(font, 0);
        std::cout << "[GUI] UTF-8 font: " << font << '\n';
        loaded = true;
        break;
      } catch (const cv::Exception&) {
      }
    }

    if (!loaded) {
      throw std::runtime_error(
          "Cannot load DejaVu Sans for UTF-8/Cyrillic HUD");
    }
  });
  return ft;
}

inline void putTextUtf8(cv::InputOutputArray img,
                        const cv::String& text,
                        cv::Point org,
                        int /*fontFace*/,
                        double fontScale,
                        cv::Scalar color,
                        int thickness = 1,
                        int lineType = cv::LINE_8,
                        bool bottomLeftOrigin = false) {
  cv::Mat mat = img.getMat();
  const int font_height =
      std::max(12, static_cast<int>(std::lround(30.0 * fontScale)));

  // FreeType uses a pixel font height rather than Hershey's relative scale.
  // The multiplier above preserves the approximate v15.3 HUD sizing.
  renderer()->putText(mat,
                      text,
                      org,
                      font_height,
                      color,
                      thickness,
                      lineType == cv::LINE_AA ? cv::LINE_AA : cv::LINE_8,
                      bottomLeftOrigin);
}

}  // namespace jtzero_utf8_v154
}  // namespace cv

// The v15.3 source and all OpenCV headers it needs have already been parsed
// above. From this point on, its cv::putText(...) calls become our UTF-8
// FreeType renderer. This also covers the inherited HUD helper txt(...).
#define putText jtzero_utf8_v154::putTextUtf8
#include "live_mono_imu_yaw_only_hud_v15_3.cpp"
#undef putText
