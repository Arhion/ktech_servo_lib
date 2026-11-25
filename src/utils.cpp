#include "utils.h"

int64_t convertAngle(float angle)
{
	return int64_t(int64_t(angle) * 1/ANGLE_RATIO);
}
