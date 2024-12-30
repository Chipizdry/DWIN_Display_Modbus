#include "uart2.h"



volatile u8 modbus_addresses[5] = {1, 2, 3, 4, 5}; // Адреса устройств
volatile u16 start_reg = 0x0001;              // Начальный регистр
volatile u16 num_reg = 4;                    // Количество регистров
volatile u8 current_dev = 0;                    // Текущее устройство для опроса

xdata volatile	u16 current_device=0;          


#if(UART2_INT_EN)
xdata u16 uart2_rx_sta;//bit15Used to mark whether a complete data packet has been received, bit[14:0] is used to store the length of the current data packetxdata u8
xdata u8  uart2_buf[UART2_PACKET_MAX_LEN+2];//Leave 2 blank characters
xdata u8  uart2_step;
xdata u8  rcv_complete=0;  // Приём завершён и обработан
idata u16 data_len=0;
//Serial port 2 interrupt service routine

void uart2_isr() interrupt 4 {
    u8 res;

    if (RI0) {  // Проверяем флаг приема данных
        RI0 = 0;  // Сбрасываем флаг приема

        res = SBUF0;  // Читаем принятый байт данных из регистра

        // Если пакет уже обработан, игнорируем дальнейшие данные
        if (uart2_rx_sta & UART2_PACKET_OK) {
            return;
        }

        // Сохраняем данные в буфер
        if (uart2_rx_sta < UART2_PACKET_MAX_LEN) {
            uart2_buf[uart2_rx_sta++] = res;
					
        } else {
            uart2_rx_sta = 0;  // Если буфер переполнен, сбрасываем
            return;
        }

        // Процесс приема данных по шагам
        if (uart2_step<data_len) {  
            uart2_step++;
        } 
				
			if(uart2_step==data_len)	{  
            uart2_rx_sta |= UART2_PACKET_OK;  // Устанавливаем флаг пакета
				  rcv_complete=1;
					uart2_step =0;
        }
    }
}
#endif


//Serial port 2 initialization
void uart2_init(u32 baud)
{
	MUX_SEL |= 0x40;//Setting bit6 to 1 means to export the uart2 interface to P0.4 and P0.5
	P0MDOUT &= 0xCF;
	P0MDOUT |= 0x10;//Set the corresponding IO port output and input
	ADCON = 0x80;//Select SREL0H:L as baud rate generator
	SCON0 = 0x50;//Accept enable and mode settings
	PCON &= 0x7F;//SMOD=0
	//Baud rate setting, the formula is:
	//SMOD=0  SREL0H:L=1024-main frequency/(64*baud rate),SMOD=1	 SREL0H:L=1024-main frequency/(32*baud rate)
	baud = 1024-(u16)(3225600.0f/baud);
	SREL0H = (baud>>8)&0xff;  
	SREL0L = baud&0xff;
	
	#if(UART2_INT_EN)
		ES0 = 1;//Interrupt enable
		EA = 1;
		//xdata variables must be initialized in functions
		uart2_rx_sta = 0;
		uart2_step = 0;
	#else
		ES0 = 0;
	#endif

}

//Send a byte
void u2_send_byte(u8 byte)
{
	ES0 = 0;//Close the serial port 2 interrupt first
	SBUF0 = byte;
	while(!TI0);
	TI0 = 0;
	#if(UART2_INT_EN)
		ES0 = 1;//Re-open interrupt
	#endif
}



//Send data
void u2_send_bytes(u8 *bytes,u16 len)
{
	u16 i;
	
	ES0 = 0;//Close the serial port 2 interrupt first
	for(i=0;i<len;i++)
	{
		SBUF0 = bytes[i];
		while(!TI0);
		TI0 = 0;
	}
	#if(UART2_INT_EN)
		ES0 = 1;//Re-open interrupt
	#endif
}


//Implement printf function with uart2 serial port
char putchar(char c)
{
	u2_send_byte(c);
	
	return c;
}




u16 calculate_crc(unsigned char *buffer, unsigned char length) {
    unsigned int temp, temp2, flag;
    unsigned int i;               // Вынесение переменной `i`
    unsigned char j;              // Вынесение переменной `j`

    temp = 0xFFFF;

    for (i = 0; i < length; i++) {
        temp = temp ^ buffer[i];
        for (j = 0; j < 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }

    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;

    return temp;
}


// Функция формирования и отправки Modbus-запроса
void modbus_request(u8 dev_addr,u8 dev_comd, u16 start_reg, u16 num_reg) {
    u8 request[8];
    u16 crc;

    // Формируем запрос Modbus
    request[0] = dev_addr;                      // Адрес устройства
    request[1] = dev_comd;               // Код функции 
    request[2] = (start_reg >> 8) & 0xFF; // Старший байт начального регистра
    request[3] = start_reg & 0xFF;        // Младший байт начального регистра
    request[4] = (num_reg >> 8) & 0xFF;  // Старший байт количества регистров
    request[5] = num_reg & 0xFF;         // Младший байт количества регистров

    // Вычисляем CRC
    crc = calculate_crc(request, 6);
    request[6] = crc & 0xFF;                   // Младший байт CRC
    request[7] = (crc >> 8) & 0xFF;            // Старший байт CRC

    // Отправляем запрос через UART
    u2_send_bytes(request, 8);
}


u8 parseModbusPacket(u8 *buffer, u16 length, ModbusPacket *parsedPacket) {
	  u16 receivedCRC;
	  u16 calculatedCRC; 
	  unsigned int m;  
    if (length < 4) {
        // Минимальная длина пакета: адрес (1 байт) + функция (1 байт) + CRC (2 байта)
        return 99 ;
    }

    // Извлекаем CRC из конца пакета
		
    receivedCRC = buffer[length - 2] | (buffer[length - 1] << 8);

    // Вычисляем CRC для проверки
    calculatedCRC = calculate_crc(buffer, length - 2);
    if (receivedCRC != calculatedCRC) {
        return 98 ; // Ошибка CRC
    }

    // Заполняем структуру пакета
    parsedPacket->rcv_address = buffer[0];
    parsedPacket->rcv_functionCode = buffer[1];
    parsedPacket->rcv_dataLength = buffer[2]; // Общая длина минус адрес, код функции, и CRC
    for (m = 0; m < parsedPacket->rcv_dataLength; m++) {
        parsedPacket->rcv_data[m] = buffer[3 + m];
    }
     return 1;
   
}



