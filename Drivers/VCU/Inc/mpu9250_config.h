#ifndef _MPU9250_CONFIG_H_
#define _MPU9250_CONFIG_H_

//#define MPU9250_USE_SPI                 1

#ifdef MPU9250_USE_SPI
    #include "spi.h"
    #define MPU9250_SPI            hspi1
    #define    MPU9250_CS_GPIO        MPU9250_CS_GPIO_Port
    #define    MPU9250_CS_PIN        MPU9250_CS_Pin
#else
    #include "i2c.h"
    #define _MPU9250_I2C        hi2c1
    #define DEVICE_ADD            208‬
#endif

#endif