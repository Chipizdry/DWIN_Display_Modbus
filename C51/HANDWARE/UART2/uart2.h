#ifndef __UART2_H__
#define __UART2_H__
#include "sys.h"
#include <stdio.h>


#define UART2_INT_EN							1				//Whether the serial port interrupt is enabled
#define UART2_PACKET_OK						0x8000  //Whether the complete packet was received
#define UART2_PACKET_LEN					0x7fff	//Length of packet
#define UART2_PACKET_MAX_LEN			100			//Defines the maximum length of the serial port package, excluding '\n' or "\r\n" end tags


//Variable declaration
extern xdata u16 uart2_rx_sta;
extern xdata u8  uart2_buf[];
extern xdata u8 rcv_complete;           // Приём завершён и обработан
extern u32 sys_tick;
extern xdata volatile	u16 current_device;    
extern idata u16 data_len;
extern  volatile u8 modbus_addresses[5]; // Адреса устройств
extern  volatile u16 start_reg;              // Начальный регистр
extern  volatile u16 num_reg;                    // Количество регистров




//Function declaration
void uart2_init(u32 baud);
void u2_send_byte(u8 byte);
void u2_send_bytes(u8 *bytes,u16 len);
void modbus_request(u8 dev_addr,u8 dev_comd, u16 start_reg, u16 num_reg);
u16 calculate_crc(unsigned char *buffer, unsigned char length);
u8 parseModbusPacket(u8 *buffer, u16 length, ModbusPacket *parsedPacket);
#endif


