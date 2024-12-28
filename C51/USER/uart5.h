

#ifndef __UART5_H__
#define __UART5_H__
#include "sys.h"
#include <stdio.h>

#define UART5_INT_EN							1
#define UART5_PACKET_OK						0x8000  //Whether the complete packet was received
#define UART5_PACKET_LEN					0x7fff	//Length of packet
#define UART5_PACKET_MAX_LEN			100			//Defines the maximum length of the serial port package, excluding '\n' or "\r\n" end tags


//Variable declaration
extern xdata u16 uart5_rx_sta;
extern xdata u8  uart5_buf[];

//Function declaration
void uart5_init(u32 baud);
void u5_send_byte(u8 byte);
void u5_send_bytes(u8 *bytes,u16 len);

#endif


