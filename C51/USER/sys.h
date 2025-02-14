#ifndef __SYS_H__
#define __SYS_H__
#include "t5los8051.h"

// Переопределение типов
typedef unsigned char   u8;
typedef unsigned short  u16;
typedef unsigned long   u32;
typedef signed char     s8;
typedef signed short    s16;
typedef signed long     s32;
typedef unsigned char bool;

 	typedef struct {
						u16 address;
						u16 command;
						u16 start_register;
						u16 num_registers;
		        u16 special_cmd;
				} ModbusRequest;
				

// Пример структуры Modbus пакета
typedef struct {
	  u16 rcv_address;        // Адрес устройства
    u16 rcv_functionCode;   // Код функции
	  u16 rcv_dataLength;    // Длина данных
    u16 rcv_data[32];      // Данные (макс. длина) 
} ModbusPacket;
				

// определение макросов
#define	WDT_ON()				MUX_SEL|=0x02		// включить сторожевой таймер
#define	WDT_OFF()				MUX_SEL&=0xFD		// отключить сторожевой таймер
#define	WDT_RST()				MUX_SEL|=0x01		// сбросить сторожевой таймер
#define	GET_VP(invar)           (u8*)&invar         // возвращает указатель на переменную


// Определение основной частоты системы и значения времени 1 мс
#define FOSC     				206438400UL
#define T1MS    				(65536-FOSC/12/1000)

// определение функций
void sys_init(void);
void sys_delay_about_ms(u16 ms);
void sys_delay_about_us(u8 us);
void sys_delay_ms(u16 ms);
void sys_read_vp(u16 addr,u8* buf,u16 len);
void sys_write_vp(u16 addr,u8* buf,u16 len);

#endif


