#include "sys.h"
#include "uart2.h"
#include <string.h>
extern  u8 modbus_addresses[5];     // Адреса устройств
extern  u16 start_reg;              // Начальный регистр
extern  u16 num_reg;                // Количество регистров

extern volatile u32 rcv_timer;
extern volatile  u32 sys_tick;
extern u8  uart2_step;

#define FIRST_TXT		 "DGUS Tool\0\0"
#define TEST_TXT		 "DGUS TEST TEXT\0\0"
#define INT_TXT		 "INERRUPT \0\0"
#define WHILE_TXT		 "WHILE___ \0\0"



// Прототип функции
void modbus_requests(ModbusRequest *requests);


void modbus_requests(ModbusRequest *requests) {
    u8 packet[8];
    u16 crc;

    // Формируем запрос Modbus
    packet[0] = requests->address;                      // Адрес устройства
    packet[1] = requests->command;                                  // Код функции (чтение регистров)
    packet[2] = (requests->start_register >> 8) & 0xFF; // Старший байт начального регистра
    packet[3] = requests->start_register & 0xFF;        // Младший байт начального регистра
    packet[4] = (requests->num_registers >> 8) & 0xFF;  // Старший байт количества регистров
    packet[5] = requests->num_registers & 0xFF;         // Младший байт количества регистров

    // Вычисляем CRC
    crc = calculate_crc(packet, 6);
    packet[7] = crc & 0xFF;                            // Младший байт CRC
    packet[6] = (crc >> 8) & 0xFF;                     // Старший байт CRC
    // Отправляем запрос через UART
    u2_send_bytes(packet, 8);
}




void main(void)
{   

// Глобальные переменные в `xdata`
idata  ModbusRequest request[6] = {
    {0x2, 0x3, 0x0000, 0x4},   // Устройство 1
    {0x4, 0x3, 0x0000, 0x2},   // Устройство 2
    {0x3, 0x3, 0x0002, 0x2},   // Устройство 3
    {0x1, 0x3, 0x0000, 0x2},   // Устройство 4
    {0x5, 0x3, 0x00FD, 0x1},  // Устройство 5
    {0x6, 0x3, 0x002F, 0x1}   // Устройство 6
};

 idata  ModbusRequest temp_request;
	u8 send_buff[8]={0, };
  u32 polling_timer=0;                    // Таймер ожидания ответа
	u8 polling_state;                     // Состояние опроса: 0 - отправка, 1 - ожидание
	u16 len;
	u16 i;
  u8 buff[48]={0, };
  u16 recv_len;
	idata u8 command_value; // Объявление переменной
	float temperature;
	u16 rawValue;
  xdata ModbusPacket receivedPacket;
	
u16 receive_cmd=0;
xdata u16 receive_adr=0;

 xdata u16 result=0;	
	sys_init();//System initialization
	
	   uart2_init(9600);//Initialize serial port 2
	
		 current_device = 0;
		 polling_state=0;
	while(1){   
		
		
		
	//	if(((sys_tick-rcv_timer)>=400000)&&(polling_state == 1)&&(rcv_complete)){uart2_rx_sta |= UART2_PACKET_OK; }; // Таймаут прерывания приёма данных 
				
					
		
		if(uart2_rx_sta & UART2_PACKET_OK)
		{
				
			len = uart2_rx_sta&UART2_PACKET_LEN;
			
			
		  sys_write_vp(0x2069, (u16*)&len, 2);
	
			recv_len = 0;
			for(i=0;i<len;i++)
			{
				recv_len += sprintf(buff+recv_len,"%02X ",(u16)uart2_buf[i]);
			}
		  sys_write_vp(0x2010,"                                ",16);
			sys_write_vp(0x2010,buff,recv_len/2+2);
			for(i=0;i<48;i++)
			{
				buff[i]=0;
			}
			
			result=parseModbusPacket(&uart2_buf,len,(ModbusPacket*)&receivedPacket);
			
			 sys_write_vp(0x2071, &result, 1);
			 if (result==1) {   
						 sys_write_vp(0x2096, "OK    \n", 4);
			 
			 switch (receivedPacket.rcv_address) {

           case 0x01:	
						 
					  // Проверяем длину данных
                    if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                        rawValue = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];
                        if (rawValue & 0x8000) { // Проверяем знак числа
                            rawValue = rawValue - 65536; // Отрицательное значение
                        }
                       temperature = rawValue / 10.0; // Масштабирование
                       sys_write_vp(0x2005,(u8*)&temperature,2);		
                       } else {
                      
                        }
                  //  break;
												
							 case 0x02:						
									

                 if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                       		
                       } else {
                      
                        }
                  //  break;							 
					 
					   //  default: break;
			 }
			 
			 
			 
			 
			 }
			 else{ sys_write_vp(0x2096, "ERROR\n", 4); }
			
			
		  	uart2_rx_sta = 0;
			  len=0;
			 
			 for(i=0;i<UART2_PACKET_MAX_LEN;i++)
			{
				uart2_buf[i]=0;
			}
			 
			 
			rcv_complete=1;
		}
	 
		
	
		
if (polling_state==0) {
	     if (current_device >= 6) {
           current_device = 0; // Сбрасываем индекс, если он выходит за границы
          }
	
					
		sys_delay_ms(5);
		temp_request = request[current_device];
		modbus_requests((ModbusRequest*)&temp_request);
		sys_write_vp(0x2000,(u8*)&current_device,1);
				
 
    command_value = temp_request.command; // Присваивание значения
    sys_write_vp(0x2001, &temp_request.command, 1); // Запись значения команды
		sys_write_vp(0x2002, &temp_request.start_register, 1); // Запись первого регистра
    data_len=(temp_request.num_registers * 2)+5;	
		sys_write_vp(0x2003,(u16*)&data_len, 2);	
    sys_write_vp(0x2004, &temp_request.address, 1);
			polling_state=1;
	    polling_timer=200000; 
			rcv_timer=sys_tick;
	     }

      polling_timer--;
		
		
		// Состояние 1: Ожидание ответа
    if (polling_state == 1) {
        // Если получен ответ
			
        if (rcv_complete==1) {
					  sys_write_vp(0x2042, "Received        \n", 9);			
            // Переход к следующему устройству
            current_device=current_device+1;
            polling_state = 0;  // Возврат в состояние отправки
					  rcv_complete=0;
					  polling_timer=200000;
        }
        // Если время ожидания истекло
         if (polling_timer ==0) {
            // Логируем таймаут (опционально)
            sys_write_vp(0x2042, "Timeout         \n", 9);
					 
					 	for(i=0;i<48;i++)
						{
							buff[i]=0;
						}
            // Переход к следующему устройству
            current_device=current_device+1;
            polling_state = 0;  // Возврат в состояние отправки
					  rcv_complete=0;
        }			
    }	
	}
		
}




