#ifndef _TCP_SERVER_H_
#define _TCP_SERVER_H_

#include <stdint.h>

int32_t tcpServerReceive(uint8_t sn, uint8_t* _receive_buf, uint16_t port);

int32_t tcpServerTransmit(uint8_t sn, uint8_t* _transmit_buf, uint16_t _len, uint16_t port);

#endif /* _TCP_SERVER_H_ */