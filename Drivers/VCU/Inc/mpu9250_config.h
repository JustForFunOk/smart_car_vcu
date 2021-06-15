#ifndef _MPU9250_CONFIG_H_
#define _MPU9250_CONFIG_H_
#include "main.h"

// Please check below configuration
#define MPU9250_USE_IIC // use IIC or not
#define MPU9250_AD0_PIN_IS_HIGH // the AD0 pin connect to VCC or not
// Please check above configuration

#ifdef MPU9250_AD0_PIN_IS_HIGH
#define DEVICE_ADDRESS        0xD2  // AD0 = 1 11010010 left shift one bit in STM32 HAL
#else
#define DEVICE_ADDRESS        0xD0  // AD0 = 0 11010000 left shift one bit in STM32 HAL
#endif

#ifdef MPU9250_USE_IIC
#define MPU9250_I2C_CHANNEL   hi2c1
#else
#define MPU9250_SPI           hspi1
#define MPU9250_CS_GPIO       MPU9250_CS_GPIO_Port
#define MPU9250_CS_PIN        MPU9250_CS_Pin
#endif

// sensor parameter
// A range


#endif  // #ifndef _MPU9250_CONFIG_H_
