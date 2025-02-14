#include "uart2.h"



volatile u8 modbus_addresses[5] = {1, 2, 3, 4, 5}; // Адреса устройств
volatile u16 start_reg = 0x0001;              // Начальный регистр
volatile u16 num_reg = 4;                    // Количество регистров
volatile u8 current_dev = 0;                    // Текущее устройство для опроса
volatile u32 rcv_timer=0;
xdata volatile	u16 current_device=0;          


#if(UART2_INT_EN)
xdata u16 uart2_rx_sta;//bit15Used to mark whether a complete data packet has been received, bit[14:0] is used to store the length of the current data packetxdata u8
xdata u8  uart2_buf[UART2_PACKET_MAX_LEN];
xdata u8  uart2_step;
xdata u8  rcv_complete=0;  // Приём завершён и обработан
idata u16 data_len=0;
//Serial port 2 interrupt service routine

void uart2_isr() interrupt 4 {
    u8 res;
	
    if (RI0) {  // Проверяем флаг приема данных
        RI0 = 0;  // Сбрасываем флаг приема
			    
         sys_tick=IDLE_TIME;
         res = SBUF0;  // Читаем принятый байт данных из регистра

        // Если пакет уже обработан, игнорируем дальнейшие данные
        if (uart2_rx_sta & UART2_PACKET_OK) {
            return;
        }

        // Сохраняем данные в буфер
        if (uart2_rx_sta < UART2_PACKET_MAX_LEN) {
            uart2_buf[uart2_rx_sta] = res;
					  uart2_rx_sta++;
					
					
					 // Проверяем второй байт (opCode) на наличие ошибки
            if ((uart2_rx_sta == 2)&&(uart2_buf[1] & 0x80)){   // Если старший бит установлен, это код ошибки
           
                    data_len = 5; // Устанавливаем длину пакета ошибки
            }
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


void uart2_reset(u32 baud)
{
    // Деинициализация UART2
    ES0 = 0;               // Отключение прерываний UART2
    SCON0 = 0x00;          // Сброс регистра управления UART
    SREL0H = 0x00;         // Сброс регистров скорости передачи
    SREL0L = 0x00;
    PCON &= ~0x80;         // Сброс SMOD
    
    // Повторная инициализация UART2
    uart2_init(baud);
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


  u8 parseModbusPacket(u8 *buffer, u16 length, ModbusPacket *parsedPacket) {
	  u16 receivedCRC;
	  u16 calculatedCRC; 
	  unsigned int m;  
		unsigned int l;
		u16 receive_adr;
		u16 receive_cmd;
		receive_adr=	buffer[0];	
	  receive_cmd=	buffer[1];	
		sys_write_vp(0x2065, &receive_adr, 1);
		sys_write_vp(0x2067, &receive_cmd, 1);
		
    if (length < 4) {
        // Минимальная длина пакета: адрес (1 байт) + функция (1 байт) + CRC (2 байта)
			 for(l=0; l<UART2_PACKET_MAX_LEN;l++) {buffer[l]=0;}
        return 99 ;
    }

    // Извлекаем CRC из конца пакета
		
    receivedCRC = buffer[length - 1] | (buffer[length - 2] << 8);

    // Вычисляем CRC для проверки
    calculatedCRC = calculate_crc(buffer, length - 2);
    if (receivedCRC != calculatedCRC) {
			 for(l=0; l<UART2_PACKET_MAX_LEN;l++) {buffer[l]=0;}
        return 98 ; // Ошибка CRC
    }

    // Заполняем структуру пакета
    parsedPacket->rcv_address = buffer[0];
    parsedPacket->rcv_functionCode = buffer[1];
		
		  if (parsedPacket->rcv_functionCode == 0x03) { // Чтение регистров
    parsedPacket->rcv_dataLength = buffer[2]; 
    for (m = 0; m < parsedPacket->rcv_dataLength; m++) {
        parsedPacket->rcv_data[m] = buffer[3 + m];
    }
		 for(l=0; l<UART2_PACKET_MAX_LEN;l++) {buffer[l]=0;}
     return 1; }  
			
		  if(parsedPacket->rcv_functionCode == 0x05) { // Запись регистров
		 
		 
		 return 1; }
			
		 if(parsedPacket->rcv_functionCode == 0x10) { // Запись регистров
		 
		 
		 return 1; }
		 
		  if(parsedPacket->rcv_functionCode == 0x0F) { // Запись регистров
		 
		 
		 return 1; }
   
}


void modbus_requests(ModbusRequest *requests,u16 *data_send, u8 data_len) {
    u8 packet[32];
    u16 crc;
    u16 i;
	  u8 len;     // Текущая длина пакета
	  u8 byte_count;
    // Формируем запрос Modbus
    packet[0] = requests->address;                      // Адрес устройства
    packet[1] = requests->command;                      // Код функции (чтение/запись  регистров)
	
	if (requests->command == 0x03) { // Чтение регистров
        packet[2] = (requests->start_register >> 8) & 0xFF; // Старший байт начального регистра
        packet[3] = requests->start_register & 0xFF;        // Младший байт начального регистра
        packet[4] = (requests->num_registers >> 8) & 0xFF;  // Старший байт количества регистров
        packet[5] = requests->num_registers & 0xFF;         // Младший байт количества регистров
        len = 6; // Длина данных для функции 3
		   
    } 
  	if (requests->command == 0x05) { // Чтение регистров
        packet[2] = (requests->start_register >> 8) & 0xFF; // Старший байт начального регистра
        packet[3] = requests->start_register & 0xFF;        // Младший байт начального регистра
        packet[4] = (requests->num_registers >> 8) & 0xFF;  // Старший байт количества регистров
        packet[5] = requests->num_registers & 0xFF;         // Младший байт количества регистров
        len = 6; // Длина данных для функции 5
		   
    } 
		
		if (requests->command == 0x06) { // Чтение регистров
        packet[2] = (requests->start_register >> 8) & 0xFF; // Старший байт начального регистра
        packet[3] = requests->start_register & 0xFF;        // Младший байт начального регистра
        packet[4] = (data_send[6] >> 8) & 0xFF;  // Старший байт количества регистров
        packet[5] = data_send[7] & 0xFF;         // Младший байт количества регистров
        len = 6; // Длина данных для функции 6
		  
    } 
		
		if (requests->command == 0x0F) { // Чтение регистров
        packet[2] = (requests->start_register >> 8) & 0xFF; // Старший байт начального регистра
        packet[3] = requests->start_register & 0xFF;        // Младший байт начального регистра
        packet[4] = (requests->num_registers >> 8) & 0xFF;  // Старший байт количества регистров
        packet[5] = requests->num_registers & 0xFF;         // Младший байт количества регистров
		   	packet[6] = (requests->special_cmd >> 8) & 0xFF;  // Старший байт количества регистров
        packet[7] = requests->special_cmd & 0xFF;         // Младший байт количества регистров
        len = 8; // Длина данных для функции 15
		  
    } 
		
		
		else if (requests->command == 0x10) { // Запись регистров
        byte_count = data_len*2;

        packet[2] = (requests->start_register >> 8) & 0xFF; // Старший байт начального регистра
        packet[3] = requests->start_register & 0xFF;        // Младший байт начального регистра
        packet[4] = (data_len >> 8) & 0xFF;                 // Старший байт количества регистров
        packet[5] = data_len & 0xFF;                        // Младший байт количества регистров
        packet[6] = byte_count & 0xFF;                      // Количество байт данных

        // Добавляем данные в пакет
        for (i = 0; i < data_len; i++) {
            packet[7 + (i * 2)] = (data_send[i] >> 8) & 0xFF;    // Старший байт данных
            packet[8 + (i * 2)] = data_send[i] & 0xFF;           // Младший байт данных
        }
   
        len = 7 + byte_count; // Длина пакета для функции 16
    }
		
		
		
		

    // Вычисляем CRC
    crc = calculate_crc(packet, len);
    packet[len+1] = crc & 0xFF;                            // Младший байт CRC
    packet[len] = (crc >> 8) & 0xFF;                     // Старший байт CRC
		 len += 2; // Увеличиваем длину на размер CRC
    // Отправляем запрос через UART
    u2_send_bytes(packet, len);
}

// Функция для установки значения переменной в определённый бит регистра
 void setBitInUint16(u16 *reg, u8 bitPos, u8 value) {
    if (bitPos < 16) { // Убедимся, что номер бита в пределах 0-15
        if (value) {
            *reg |= (1 << bitPos); // Установить бит
        } else {
            *reg &= ~(1 << bitPos); // Сбросить бит
        }
    }
}