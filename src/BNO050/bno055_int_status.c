// Read INT_STATUS register (0x37) from BNO055
// Returns the value of INT_STATUS (see datasheet for bit meanings)
#include "bno055.h"

uint8_t bno055_getIntStatus(void) {
    bno055_setPage(0); // INT_STATUS is on page 0
    uint8_t status = 0;
    bno055_readData(BNO055_INT_STATUS, &status, 1);
    return status;
}
