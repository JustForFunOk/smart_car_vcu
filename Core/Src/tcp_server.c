#include "loopback.h"
#include "socket.h"
#include "wizchip_conf.h"
#include <stdio.h>

#define _W5500_DEBUG_

int32_t tcpServerReceive(uint8_t sn, uint8_t* _transmit_buf, uint16_t port)
{
    int32_t ret;
    uint16_t receive_size = 0, sentsize=0;

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

                ret = recv(sn, _transmit_buf, receive_size);  // receive data

                if(ret <= 0) return ret;  // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
                receive_size = (uint16_t) ret;
            }
            break;

        case SOCK_CLOSE_WAIT :
#ifdef _LOOPBACK_DEBUG_
            //printf("%d:CloseWait\r\n",sn);
#endif
            if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _LOOPBACK_DEBUG_
            printf("%d:Socket Closed\r\n", sn);
#endif
            break;

        case SOCK_INIT :
#ifdef _LOOPBACK_DEBUG_
            printf("%d:Listen, wait for client connect, port [%d]\r\n", sn, port);
#endif
            if( (ret = listen(sn)) != SOCK_OK) return ret;
            break;

        case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
            //printf("%d:TCP server loopback start\r\n",sn);
#endif
            if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _LOOPBACK_DEBUG_
            //printf("%d:Socket opened\r\n",sn);
#endif
            break;

        default:
            break;
    }
    return 1;
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
#ifdef _LOOPBACK_DEBUG_
            //printf("%d:CloseWait\r\n",sn);
#endif
            if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _LOOPBACK_DEBUG_
            printf("%d:Socket Closed\r\n", sn);
#endif
            break;

        case SOCK_INIT :
#ifdef _LOOPBACK_DEBUG_
            printf("%d:Listen, wait for client connect, port [%d]\r\n", sn, port);
#endif
            if( (ret = listen(sn)) != SOCK_OK) return ret;
            break;

        case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
            //printf("%d:TCP server loopback start\r\n",sn);
#endif
            if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _LOOPBACK_DEBUG_
            //printf("%d:Socket opened\r\n",sn);
#endif
            break;

        default:
            break;
    }
    return 1;
}