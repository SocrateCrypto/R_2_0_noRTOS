#include "IMU_data.h"
#include <cmath>



unwrap_t up[3] = { { 0.0000, 0 }, { 0.0000, 0 }, { 0.0000, 0 } };






float unwrap(unwrap_t *up, float value)
{
    uint8_t z = 0;
    if (fabs(value - up->old_degs) > 180) {

        if (value - up->old_degs > 0) {
            z = 1;
        } else {
            z = 2;
        }
    }
    up->old_degs = value;

    if (z == 1) {
        up->rotations--;
    } else if (z == 2) {
        up->rotations++;
    }

    if (up->rotations == 0) {
        return value;
    } else {
        return value + (360.0f * (float) up->rotations);
    }

}
