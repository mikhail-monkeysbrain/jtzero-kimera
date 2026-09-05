#include "v25_parts/live_mono_imu_500mm_repeat_hud_v25_part01.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part02.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part03.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part04.inc"
#define JTZERO_SUMMARY_TITLE "500 MM A-B-A x2 V25 CLOSURE TEST"
#define JTZERO_SUMMARY_FUSION "pure FRD->FLU, ZXY OFF, gravity feedback OFF"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part05.inc"
#undef JTZERO_SUMMARY_FUSION
#undef JTZERO_SUMMARY_TITLE
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part06a.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part06b.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07a.inc"
#define JTZERO_STRICT_STALL_NS 500000000LL
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc"
#undef JTZERO_STRICT_STALL_NS
#include "v25_parts/live_mono_imu_500mm_repeat_hud_v25_part08a.inc"
#include "v25_parts/live_mono_imu_500mm_repeat_hud_v25_part08b.inc"
