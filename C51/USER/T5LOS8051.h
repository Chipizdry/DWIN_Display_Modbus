#ifndef __T5LOS8051_H__
#define __T5LOS8051_H__


sfr	P0		=	0x80;		/********PO порт*******/
sfr	SP		=	0x81;		/******** stack pointer / указатель стека *******/
sfr DPL		=	0x82;		/********DPTR data pointer / Указатель данных DPTR*******/
sfr DPH		=	0x83;		/********DPTR data pointer / Указатель данных DPTR*******/
sfr PCON	=	0x87;		/********.7 UART2 baud rate setting / .7 Настройка скорости передачи UART2*******/
sfr TCON	=	0x88;		/********T0 T1 Регистр управления*******/

sbit	TF1 =	TCON^7;		/********T1 interrupt trigger / Триггер прерывания T1*******/
sbit	TR1	=	TCON^6;		
sbit	TF0	=	TCON^5;		/********Триггер прерывания T0*******/
sbit	TR0	=	TCON^4;
sbit	IE1	=	TCON^3;		/********Внешнее прерывание 1*******/
sbit	IT1	=	TCON^2;		/********Режим запуска внешнего прерывания 1. 0: запуск по низкому уровню 1: запуск по заднему фронту*******/
sbit	IE0	=	TCON^1;		/********Внешнее прерывание 0*******/
sbit	IT0	=	TCON^0;		/********Режим запуска внешнего прерывания 0. 0: запуск по низкому уровню 1: запуск по заднему фронту*******/

sfr	TMOD	=	0x89;		/********T0 Выбор режима T1, такой же, как 8051*******/
sfr	TH0 	=	0x8C;		
sfr TL0 	=	0x8A;
sfr TH1 	=	0x8D;
sfr TL1 	=	0x8B;

sfr CKCON	=	0x8E;		/********CPU работает*******/
sfr	P1		=	0x90;
sfr	DPC		=	0x93;		/********После инструкции MOVX режим изменения. DPTR 0: без изменений, 1: +1,  2: -1 *******/
sfr PAGESEL	=	0x94;		/********Должно быть 0x01*******/
sfr	D_PAGESEL	=	0x95;	/********должно быть 0x02*******/

sfr SCON0	=	0x98;		/********Интерфейс управления UART2, такой же, как 8051*******/
sbit	TI0	=	SCON0^1;
sbit	RI0	=	SCON0^0;
sfr	SBUF0	=	0x99;		/********Интерфейс передачи данных приемопередатчика UART2*******/
sfr	SREL0H	=	0xBA;		/********Установите скорость передачи данных, когда ADCON равен 0x80.*******/
sfr	SREL0L	=	0xAA;

sfr	SCON1	=	0x9B;		/********Интерфейс управления UART3*******/
sfr	SBUF1	=	0x9C;
sfr	SREL1H	=	0xBB;
sfr	SREL1L	=	0x9D;

sfr	IEN2	=	0x9A;		/********Контроллер разрешения прерываний SFR .7~.1 должен быть записан 0.0 Бит управления разрешением прерывания UART3*******/
sfr	P2		=	0xA0;
sfr	IEN0	=	0xA8;		/********Контроллер разрешения прерываний 0*******/
sbit	EA	=	IEN0^7;		/********главный управляющий бит прерывания*******/
sbit	ET2	=	IEN0^5;		/********Биты управления прерыванием таймера 2*******/
sbit	ES0	=	IEN0^4;		/********UART2*******/
sbit	ET1	=	IEN0^3;		/********T1*******/
sbit	EX1	=	IEN0^2;		/********Внешнее прерывание 1*******/
sbit	ET0	=	IEN0^1;		/********T0*******/
sbit	EX0	=	IEN0^0;		/********Внешнее прерывание 0*******/

sfr	IP0		=	0xA9;				/********Контроллер приоритета прерывания 0*******/
sfr	P3		=	0xB0;
sfr	IEN1	=	0xB8;				/********Прерывание включения контроллера приема******/
sbit	ES3R	=	IEN1^5;			/*****Управляющий бит разрешения прерывания приема UART5****/
sbit	ES3T	=	IEN1^4;			/*****Управляющий бит разрешения прерывания приема UART5****/
sbit	ES2R	=	IEN1^3;			/*****Управляющий бит разрешения прерывания приема UART4****/
sbit	ES2T	=	IEN1^2;			/*****Управляющий бит разрешения прерывания приема UART4****/
sbit	ECAN	=	IEN1^1;			/********Бит управления разрешением прерывания CAN******/

sfr	IP1		=	0xB9;				/********Контроллер приоритета прерывания 0*******/
sfr	IRCON2	=	0xBF;
sfr	IRCON 	=	0xC0;
sbit	TF2	=	IRCON^6;			/********Флаг срабатывания прерывания T2*******/
sfr	T2CON	=	0xC8;				/********Регистр управления T2********/
sbit	TR2	=	T2CON^0;			/***********Т2 включить***********/
sfr	TRL2H	=	0xCB;
sfr	TRL2L	=	0xCA;
sfr	TH2 	=	0xCD;
sfr	TL2 	=	0xCC;

sfr	PSW		=	0xD0;
sbit	CY	=	PSW^7;
sbit	AC	=	PSW^6;
sbit	F0	=	PSW^5;
sbit	RS1	=	PSW^4;
sbit	RS0	=	PSW^3;
sbit	OV	=	PSW^2;
sbit	F1	=	PSW^1;
sbit	P	=	PSW^0;
sfr	ADCON	=	0xD8;
sfr	ACC		=	0xE0;
sfr	B 		=	0xF0;

/******Определение аппаратного расширения*********/
/******Доступ к RAM(памяти переменных) DGUS*********/
sfr	RAMMODE	=	0xF8;				/******Регистр управления интерфейсом доступа к памяти переменных DGUS*********/
sbit	APP_REQ	=	RAMMODE^7;
sbit	APP_EN	=	RAMMODE^6;
sbit	APP_RW	=	RAMMODE^5;
sbit	APP_ACK	=	RAMMODE^4;
sfr ADR_H	=	0xF1;
sfr ADR_M	=	0xF2;
sfr ADR_L	=	0xF3;
sfr ADR_INC	=	0xF4;
sfr DATA3	=	0xFA;
sfr DATA2	=	0xFB;
sfr DATA1	=	0xFC;
sfr DATA0	=	0xFD;


//UART4
sfr	SCON2T	=	0x96;					/******Управление отправкой UART4********/
sfr	SCON2R	=	0x97;					/******UART4 получает управление*********/
sfr	BODE2_DIV_H	=	0xD9;				/******Настройка скорости передачи данных********/
sfr	BODE2_DIV_L	=	0xE7;
sfr	SBUF2_TX	=	0x9E;				/******Интерфейс отправки данных UART4********/
sfr	SBUF2_RX	=	0x9F;				/******Интерфейс приема данных UART4*********/

//UART5
sfr	SCON3T	=	0xA7;					
sfr	SCON3R	=	0xAB;
sfr	BODE3_DIV_H	=	0xAE;
sfr	BODE3_DIV_L	=	0xAF;
sfr	SBUF3_TX	=	0xAC;
sfr	SBUF3_RX	=	0xAD;

//CAN связь
sfr	CAN_CR	=	0x8F;
sfr	CAN_IR	=	0x91;
sfr	CAN_ET	=	0xE8;

//GPIO
sfr	P0MDOUT	=	0xB7;
sfr	P1MDOUT	=	0xBC;
sfr	P2MDOUT	=	0xBD;
sfr	P3MDOUT	=	0xBE;
sfr	MUX_SEL	=	0xC9;
sfr	PORTDRV	=	0xF9;				/******мощность выходного привода / output drive strength*********/

//MAC&DIV
sfr	MAC_MODE	=	0xE5;
sfr	DIV_MODE	=	0xE6;

//Интерфейс расширения SFR       SFR expansion interface
sfr	EXADR	=	0xFE;
sfr	EXDATA	=	0xFF;





#endif


