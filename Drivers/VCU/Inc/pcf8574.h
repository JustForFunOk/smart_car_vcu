#ifndef _PCF8574_H_
#define _PCF8574_H_

#include "main.h"

#define PCF8574_WRITE_IIC_ADDRESS  0x40  // Address: 010 0000 (A0 & A1 & A2 connect to GND)
#define PCF8574_IIC_CHANNEL        hi2c2

#define LED_ALL_OFF                0xff
#define LED_ALL_ON                 0x00

/**
  * @brief PCF8574 SetLedStatus
  * @param _receive_led_status the 8-bit data to control led status.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef PCF8574_SetLedStatus(uint8_t _receive_led_status);

#endif  // #ifndef _PCF8574_H_