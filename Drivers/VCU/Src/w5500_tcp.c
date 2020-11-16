#include "w5500_tcp.h"
#include "wizchip_conf.h"
#include "socket.h"  // ctlnetwork reg_wizchip_cs_cbfunc reg_wizchip_spi_cbfunc
#include "main.h"  // HAL_*
#include <stdio.h>
#include <string.h>  // memcmp

#define _W5500_DEBUG_
#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE			2048
#endif

void W5500_Select(void)
{
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void)
{
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void W5500_Restart(void)
{
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);  // delay 1ms
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1600);  // delay 1600ms
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len)
{
  HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len)
{
  HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void)
{
  uint8_t byte;
  W5500_ReadBuff(&byte, sizeof(byte));
  return byte;
}

void W5500_WriteByte(uint8_t byte)
{
  W5500_WriteBuff(&byte, sizeof(byte));
}

wiz_NetInfo gSetNetInfo ={
  .mac  = {0x00, 0x08, 0xdc, 0x11, 0x11, 0x11},
  .ip   = {192, 168, 3, 99},
  .sn   = {255, 255, 255, 0},
  .gw   = {192, 168, 3, 1},
  .dns  = {144, 144, 144, 144},
  .dhcp = NETINFO_STATIC};

wiz_NetInfo gGetNetInfo;

enum Status
{
  Failed = 0,
  Success = 1
};

/**
 * @brief valid the result of set net info
 * @return 1: Success
 *         0: Failed
*/
uint8_t validSetNetInfoResult(wiz_NetInfo* _set, wiz_NetInfo* _get)
{
  return (!memcmp(_set, _get, sizeof(wiz_NetInfo)));  // if same, memcmp return 0
}

void W5500_Init()
{
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);

    W5500_Restart();  // hardware restart through RESET pin

    ctlnetwork(CN_SET_NETINFO, (void*)&gSetNetInfo);  // set net info
    // maybe need delay
    ctlnetwork(CN_GET_NETINFO, (void*)&gGetNetInfo);  // get net info

    if(Success == validSetNetInfoResult(&gSetNetInfo, &gGetNetInfo))  // compare
    {
        printf("Net info set success!\n");
    }
    else
    {
        printf("Net info set failed!\n");
        // do something
    }

    // W5500 has 8 channel, 32k buffer, 2 means 2KBytes
    uint8_t buffer_size_8channel_tx_rx[16] = {2, 2, 2, 2, 2, 2, 2, 2,  // 8 channel tx
                                            2, 2, 2, 2, 2, 2, 2, 2}; // 8 channel rx
    if(ctlwizchip(CW_INIT_WIZCHIP,(void*)buffer_size_8channel_tx_rx))
    {
        // failed
        printf("buffer size set failed!\n");
    }
}



int32_t tcpServerReceive(uint8_t sn, uint8_t* _receive_buf, uint16_t port)
{
    int32_t ret;
    uint16_t receive_size = 0;

#ifdef _W5500_DEBUG_
    uint8_t client_ip[4];
    uint16_t client_port;
#endif

    switch(getSn_SR(sn))  // [Sn_SR] Socket n Status
    {
        case SOCK_ESTABLISHED :
            if(getSn_IR(sn) & Sn_IR_CON)  // [Sn_IR] Socket n Interrupt
            {  // Connect for the first time
#ifdef _W5500_DEBUG_
                getSn_DIPR(sn, client_ip);  // [Sn_DIPR] Socket n Destination IP Address
                client_port = getSn_DPORT(sn);  // [Sn_DPORT] Socket n Destination Port
                printf("%d:Client Connected - %d.%d.%d.%d : %d\r\n",sn, client_ip[0], client_ip[1], client_ip[2], client_ip[3], client_port);
#endif
                setSn_IR(sn,Sn_IR_CON);
            }

            // [Sn_RX_RSR] Socket n RX Received Size
            if((receive_size = getSn_RX_RSR(sn)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
            {
                if(receive_size > DATA_BUF_SIZE) receive_size = DATA_BUF_SIZE;

                ret = recv(sn, _receive_buf, receive_size);  // receive data

                if(ret <= 0) return ret;  // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
                receive_size = (uint16_t) ret;
            }
            break;

        case SOCK_CLOSE_WAIT :
#ifdef _W5500_DEBUG_
            //printf("%d:CloseWait\r\n",sn);
#endif
            if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _W5500_DEBUG_
            printf("%d:Socket Closed\r\n", sn);
#endif
            break;

        case SOCK_INIT :
#ifdef _W5500_DEBUG_
            printf("%d:Listen, wait for client connect, port [%d]\r\n", sn, port);
#endif
            if( (ret = listen(sn)) != SOCK_OK) return ret;
            break;

        case SOCK_CLOSED:
#ifdef _W5500_DEBUG_
            //printf("%d:TCP server loopback start\r\n",sn);
#endif
            if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _W5500_DEBUG_
            //printf("%d:Socket opened\r\n",sn);
#endif
            break;

        default:
            break;
    }
    return ret;
}

int32_t tcpServerTransmit(uint8_t sn, uint8_t* _transmit_buf, uint16_t _len, uint16_t port)
{
    int32_t ret;
    uint16_t sent_size=0;

#ifdef _W5500_DEBUG_
    uint8_t client_ip[4];
    uint16_t client_port;
#endif

    switch(getSn_SR(sn))  // [Sn_SR] Socket n Status
    {
        case SOCK_ESTABLISHED :
            if(getSn_IR(sn) & Sn_IR_CON)  // [Sn_IR] Socket n Interrupt
            {  // Connect for the first time
#ifdef _W5500_DEBUG_
                getSn_DIPR(sn, client_ip);  // [Sn_DIPR] Socket n Destination IP Address
                client_port = getSn_DPORT(sn);  // [Sn_DPORT] Socket n Destination Port
                printf("%d:Connected - %d.%d.%d.%d : %d\r\n",sn, client_ip[0], client_ip[1], client_ip[2], client_ip[3], client_port);
#endif
                setSn_IR(sn,Sn_IR_CON);
            }

            sent_size = 0;
            while(_len != sent_size)  // TODO: add timeout
            {
                ret = send(sn, _transmit_buf+sent_size, _len-sent_size);
                if(ret < 0)
                {
                    close(sn);
                    return ret;
                }
                sent_size += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
            break;

        case SOCK_CLOSE_WAIT :
#ifdef _W5500_DEBUG_
            //printf("%d:CloseWait\r\n",sn);
#endif
            if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _W5500_DEBUG_
            printf("%d:Socket Closed\r\n", sn);
#endif
            break;

        case SOCK_INIT :
#ifdef _W5500_DEBUG_
            printf("%d:Listen, wait for client connect, port [%d]\r\n", sn, port);
#endif
            if( (ret = listen(sn)) != SOCK_OK) return ret;
            break;

        case SOCK_CLOSED:
#ifdef _W5500_DEBUG_
            //printf("%d:TCP server loopback start\r\n",sn);
#endif
            if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _W5500_DEBUG_
            //printf("%d:Socket opened\r\n",sn);
#endif
            break;

        default:
            break;
    }
    return 1;
}