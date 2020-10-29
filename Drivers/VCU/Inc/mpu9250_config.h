#ifndef _MPU9250_CONFIG_H_
#define _MPU9250_CONFIG_H_
#include "main.h"
// use IIC or not
#define MPU9250_USE_IIC

#ifdef MPU9250_USE_IIC
#define MPU9250_I2C_CHANNEL   hi2c1
// AD = 0
#define DEVICE_ADDRESS        0xD0
#else
#define MPU9250_SPI           hspi1
#define MPU9250_CS_GPIO       MPU9250_CS_GPIO_Port
#define MPU9250_CS_PIN        MPU9250_CS_Pin
#endif  // #ifdef MPU9250_USE_IIC

// sensor parameter
// A range


#endif  // #ifndef _MPU9250_CONFIG_H_
