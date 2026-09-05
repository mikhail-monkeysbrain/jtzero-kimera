#include "v24_parts/live_mono_imu_500mm_repeat_hud_v24_part01.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part02.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part03.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part04.inc"
#define JTZERO_SUMMARY_TITLE "500 MM x2 V24 TILT-GATED RESULT"
#define JTZERO_SUMMARY_FUSION "pure FRD->FLU, ZXY OFF, gravity feedback OFF, FC relative tilt <=0.5deg"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part05.inc"
#undef JTZERO_SUMMARY_FUSION
#undef JTZERO_SUMMARY_TITLE
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part06a.inc"
#define JTZERO_B_FIRST_HUD
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part06b.inc"
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07a.inc"
#undef JTZERO_B_FIRST_HUD
#define JTZERO_STRICT_STALL_NS 500000000LL
#include "v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc"
#undef JTZERO_STRICT_STALL_NS
#include "v24_parts/live_mono_imu_500mm_repeat_hud_v24_part08a.inc"
#include "v24_parts/live_mono_imu_500mm_repeat_hud_v24_part08b.inc"
