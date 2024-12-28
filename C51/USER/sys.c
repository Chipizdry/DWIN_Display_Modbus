#include "sys.h"


static idata u16 delay_tick = 0; // для точной задержки

volatile  u32 sys_tick = 0; 

// инициализация основных регистров
void sys_cpu_init()
{
	EA = 0; // откл глоб прерывания
	RS0 = 0;
	RS1 = 0;

	CKCON = 0x00;
	T2CON = 0x70;
	DPC = 0x00;
	PAGESEL = 0x01;
	D_PAGESEL = 0x02; //DATA RAM  0x8000-0xFFFF
	MUX_SEL = 0x00;   // UART2, UART2 on, WDT off
	RAMMODE = 0x00;
	PORTDRV = 0x01;   // Drive Strength +/-8mA
	IEN0 = 0x00;      // turn off all interrupts   /   отключить все прерывания
//	IEN1 = 0x00;
	IEN1 |= 0x30;  // Разрешаем прерывания для UART5 (бит 4 для приема, бит 5 для передачи)
	IEN2 = 0x00;
	IP0 = 0x00;       // Interrupt priority default /   Приоритет прерывания по умолчанию
	IP1 = 0x00;

	WDT_OFF();      	// close the door dog    /   отключить watchdog
	
	  // Включаем прерывания для UART5
    ES3R = 1;          // Разрешить прерывание для приема UART5
    ES3T = 1;          // Разрешить прерывание для передачи UART5
    EA = 1;            // Включаем глобальные прерывания
    
}


// Таймер 2 инициализирован, а временной интервал равен 1 мс.
void sys_timer2_init()
{
	T2CON = 0x70;
	TH2 = 0x00;
	TL2 = 0x00;

	TRL2H = 0xBC;	// таймер 1 мс
	TRL2L = 0xCD;       

	IEN0 |= 0x20;	// запуск таймера 2
	TR2 = 0x01;
	EA = 1; // вкл глоб прерывания
}


// инициализация системы
void sys_init()
{
	sys_cpu_init(); // инициализация основных регистров
	sys_timer2_init(); // Инициализация таймера 2
}

// грубая(не точная) задержка, в мс
// Если уровень оптимизации изменен, параметры внутри этой функции необходимо повторно отладить
void sys_delay_about_ms(u16 ms)
{
	u16 i,j;
	for(i=0;i<ms;i++)
			for(j=0;j<3000;j++);    
}

// грубая(не точная) задержка, us
// Если уровень оптимизации изменен, параметры внутри этой функции необходимо повторно отладить
void sys_delay_about_us(u8 us)
{
	u8 i,j;
	for(i=0;i<us;i++)
			for(j=0;j<5;j++);    
}


// точная задержка в мс использущая таймер 2
void sys_delay_ms(u16 ms)
{
	delay_tick = ms;
	while(delay_tick);
}


// Чтение данных переменной VP в DGUS
// addr: адрес, напрямую переданный в DGUS
// buf: буфер
// len: количество прочитанных слов, одно слово равно 2 байтам
void sys_read_vp(u16 addr, u8* buf, u16 len)
{   
	u8 i; 
	
	i = (u8)(addr&0x01);
	addr >>= 1;
	ADR_H = 0x00;
	ADR_M = (u8)(addr>>8);
	ADR_L = (u8)addr;
	ADR_INC = 0x01;
	RAMMODE = 0xAF;
	while(APP_ACK==0);
	while(len>0)
	{   
		APP_EN=1;
		while(APP_EN==1);
		if((i==0)&&(len>0))   
		{   
			*buf++ = DATA3;
			*buf++ = DATA2;                      
			i = 1;
			len--;	
		}
		if((i==1)&&(len>0))   
		{   
			*buf++ = DATA1;
			*buf++ = DATA0;                      
			i = 0;
			len--;	
		}
	}
	RAMMODE = 0x00;
}

// Запись данных переменной VP в DGUS
// addr: адрес, напрямую переданный в DGUS
// buf: буфер
// len: количество слов данных для отправки, одно слово равно 2 байтам
void sys_write_vp(u16 addr, u8* buf, u16 len)
{   
	u8 i;  
	
	i = (u8)(addr&0x01);
	addr >>= 1;
	ADR_H = 0x00;
	ADR_M = (u8)(addr>>8);
	ADR_L = (u8)addr;    
	ADR_INC = 0x01;
	RAMMODE = 0x8F;
	while(APP_ACK==0);
	if(i && len>0)
	{	
		RAMMODE = 0x83;	
		DATA1 = *buf++;		
		DATA0 = *buf++;	
		APP_EN = 1;		
		len--;
	}
	RAMMODE = 0x8F;
	while(len>=2)
	{	
		DATA3 = *buf++;		
		DATA2 = *buf++;
		DATA1 = *buf++;		
		DATA0 = *buf++;
		APP_EN = 1;		
		len -= 2;
	}
	if(len)
	{	
		RAMMODE = 0x8C;
		DATA3 = *buf++;		
		DATA2 = *buf++;
		APP_EN = 1;
	}
	RAMMODE = 0x00;
} 


// Процедура обслуживания прерывания таймера 2
void sys_timer2_isr()	interrupt 5
{
	TF2=0; // Очистить бит флага прерывания Таймера 2
	if(sys_tick<0xFFFFFFFF){
	sys_tick++;}
	if(sys_tick==0xFFFFFFFF){sys_tick=0;}
	// обработка точной задержки
	if(delay_tick)
		delay_tick--;
}



