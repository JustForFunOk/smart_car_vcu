#ifndef _W5500_TCP_H_
#define _W5500_TCP_H_

#include <stdint.h>  // uint8_t

// transmit configuration
#define TRANSMIT_DATA_LENGTH  18  // review this value after change message fromat
#define TRANSMIT_PERIOD_MS    100  // 10Hz  20  // 50Hz
// receive configuration


#define TCP_SERVER_PORT 5000
// uint16_t destport = 5000;
uint8_t receive_buff[2048];
// uint8_t transmit_buff[] = "hello client\n";
uint8_t transmit_data[TRANSMIT_DATA_LENGTH];

void W5500_Init();

int32_t tcpServerReceive(uint8_t sn, uint8_t* _receive_buf, uint16_t port);

int32_t tcpServerTransmit(uint8_t sn, uint8_t* _transmit_buf, uint16_t _len, uint16_t port);

#endif
