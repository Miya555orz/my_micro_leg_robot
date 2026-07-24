#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c2;
void MX_I2C2_Init(void);

#ifdef __cplusplus
}
#endif

#endif
