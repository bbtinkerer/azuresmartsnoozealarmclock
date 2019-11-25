#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <applibs/i2c.h>
#include <applibs/log.h>
#include <hw/avnet_mt3620_sk.h>
#include "i2c.h"

int i2cFd = -1;

int initI2c(void) {
	if ((i2cFd = I2CMaster_Open(AVNET_MT3620_SK_ISU2_I2C)) < 0) {
		Log_Debug("Error: I2CMaster_Open: i2cFd=%d (%s).\n", i2cFd, strerror(errno));
		return -1;
	}

	if (I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD) != 0) {
		Log_Debug("Error: I2CMaster_SetBusSpeed: errno=%d (%s).\n", errno, strerror(errno));
		return -1;
	}

	if (I2CMaster_SetTimeout(i2cFd, 100) != 0) {
		Log_Debug("Error: I2CMaster_SetTimeout: errno=%d (%s).\n", errno, strerror(errno));
		return -1;
	}
	return 0;
}

void closeI2c(void) {
	//CloseFdAndPrintError(i2cFd, "i2c");
	if (i2cFd >= 0) {
		int result = close(i2cFd);
		if (result != 0) {
			Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", "i2c", strerror(errno), errno);
		}
	}
}