#pragma once

#include <stdbool.h>

int initI2c(void);
void closeI2c(void);

extern int i2cFd;