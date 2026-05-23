#pragma once

#include <cmath>

#ifndef PX4_ISFINITE
#define PX4_ISFINITE(x) std::isfinite(x)
#endif

#ifndef M_PI_F
#define M_PI_F 3.14159265f
#endif

#ifndef M_TWOPI_F
#define M_TWOPI_F 6.28318531f
#endif

#ifndef M_PI_2_F
#define M_PI_2_F 1.57079632f
#endif

#ifndef M_PI_4_F
#define M_PI_4_F 0.78539816f
#endif

#ifndef M_3PI_4_F
#define M_3PI_4_F 2.35619449f
#endif

#ifndef M_DEG_TO_RAD_F
#define M_DEG_TO_RAD_F 0.0174532925f
#endif

#ifndef M_RAD_TO_DEG_F
#define M_RAD_TO_DEG_F 57.2957795f
#endif
