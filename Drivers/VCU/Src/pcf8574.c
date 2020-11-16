#include "pcf8574.h"


HAL_StatusTypeDef PCF8574_SetLedStatus(uint8_t _receive_led_status)
{
    uint8_t iic_read_status = HAL_ERROR;

    iic_read_status = HAL_I2C_Master_Transmit(&PCF8574_IIC_CHANNEL, PCF8574_WRITE_IIC_ADDRESS, &_receive_led_status, 1, 1000);
    // HAL_Delay(1); // wait 1ms for ready

    return iic_read_status;
}