#pragma once

// Stand-in for PX4's px4_defines.h (it pulls in the board configuration). PX4's motion_planning
// (PositionSmoothing) needs only PX4_ISFINITE and the float constants from the adapter's shim.
#include <px4_platform_common/defines.h>
