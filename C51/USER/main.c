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
#define BOUDRATE 9600
#define DEVICES 8
#define BTN_VAL_ADDR 3000
#define POLLING_TIME 160000
#define POWER_TIME 900000

void main(void)
{   

// Глобальные переменные в `xdata`
idata  ModbusRequest request[DEVICES] = {
    {0x1, 0x5,   0x0000, 0x0,0x0000},   // Устройство 1
    {0x2, 0x3,   0x0000, 0x4,0x0000},   // Устройство 2
    {0x2, 0x10,  0x0000, 0x1,0x0000},   // Устройство 3
		{0x3, 0x3,   0x0000, 0x1,0x0000},   // Устройство 4
    {0x4, 0x3,   0x0000, 0x1,0x0000},   // Устройство 5
    {0x5, 0x3,   0x0000, 0x1,0x0000},   // Устройство 6
		{0x6, 0x3,   0x0000, 0x1,0x0000},   // Устройство 7
		{0x7, 0x3,   0x0000, 0x1,0x0000}    // Устройство 8
};

 idata  ModbusRequest temp_request;
	u8 send_buff[8]={0,};
  u32 polling_timer=0;                    // Таймер ожидания ответа
	u8 polling_state;                     // Состояние опроса: 0 - отправка, 1 - ожидание
	u8 power_delay;
	u32 power_timer;
	u16 len;
	u16 i;
  u8 buff[48]={0, };
  idata u16 send_reg[8]={0,0,0,0,0,0,0,0 };
	idata u16 btn_val;
	u16 pwm_width;
	u16 coil_1;
	u16 coil_2;
	u16 coil_3;
	u16 coil_4;
	
	u16 icon_1;
	u16 icon_2;
	u16 icon_3;
	u16 icon_4;
	u16 pwr_btn;
	u16 pwr_icon;
	u16 auto_manual;
  u16 recv_len;
	idata u8 command_value; 
	float temperature;
	float temperature_1;
	float temperature_2;
	float temperature_3;
	float temperature_4;
	u16 rawValue;
  xdata ModbusPacket receivedPacket;
	u16 freq;
	u16 rpm;
  u16 receive_cmd=0;
  xdata u16 receive_adr=0;
	

     xdata u16 result=0;	
     sys_init();//System initialization
	   uart2_init(BOUDRATE);//Initialize serial port 2
		 current_device = 0;
		 polling_state=0;
	   sys_tick=IDLE_TIME;
	   power_delay=0;
	   power_timer=0;
	  
	while(1){   
			
		if((sys_tick==0)&&(polling_state == 1)&&(uart2_rx_sta)){uart2_rx_sta |= UART2_PACKET_OK; }; // Таймаут прерывания приёма данных 
					
		if(uart2_rx_sta & UART2_PACKET_OK)
		{
				
			len = uart2_rx_sta & UART2_PACKET_LEN;
				
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
			sys_tick=IDLE_TIME;
			result=parseModbusPacket(&uart2_buf,len,(ModbusPacket*)&receivedPacket);		
			 sys_write_vp(0x2071, &result, 1);
			 if (result==1) {   
						 sys_write_vp(0x2096, "OK    \n", 4);
			 
			 switch (receivedPacket.rcv_address) {

           case 0x01:		 
					  // Проверяем длину данных
               /*     if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                        rawValue = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];
                        if (rawValue & 0x8000) { // Проверяем знак числа
                            rawValue = rawValue - 65536; // Отрицательное значение
                        }
                       temperature = rawValue / 10.0; // Масштабирование
                       sys_write_vp(0x2005,(u8*)&temperature,2);	
                       rawValue =	0;											
                       } else {
                      
                        } */
                    break;
												
						case 0x02:						
									

                 if (receivedPacket.rcv_dataLength >= 2) {
                      
									  // Извлекаем данные (первый регистр)
                       freq = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];  
									     rpm =(receivedPacket.rcv_data[4] << 8) | receivedPacket.rcv_data[5];
									      sys_write_vp(0x2081,(u16*)&rpm,1);	
                        sys_write_vp(0x2007,(u16*)&freq,2);			
                       } else {
												 	sys_write_vp(0x2096, "DATA_ERR\n", 6);
												 
                         break;
                        }
                    break;							 
					 
		          case 0x03:		 
					    // Проверяем длину данных
                    if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                        rawValue = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];
                        if (rawValue & 0x8000) { // Проверяем знак числа
                            rawValue = rawValue - 65536; // Отрицательное значение
                        }
                       temperature_1 = rawValue / 10.0; // Масштабирование
                       sys_write_vp(0x2085,(u8*)&temperature_1,2);	
                       rawValue =	0;												
                       } else {
                      
                        }
                    break;										
												
												
									case 0x04:		 
					        // Проверяем длину данных
                    if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                        rawValue = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];
                        if (rawValue & 0x8000) { // Проверяем знак числа
                            rawValue = rawValue - 65536; // Отрицательное значение
                        }
                       temperature_2 = rawValue / 10.0; // Масштабирование
                       sys_write_vp(0x2087,(u8*)&temperature_2,2);	
                       rawValue =	0;												
                       } else {
                      
                        }
                    break;										
												
												
							    case 0x05:		 
					        // Проверяем длину данных
                    if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                        rawValue = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];
                        if (rawValue & 0x8000) { // Проверяем знак числа
                            rawValue = rawValue - 65536; // Отрицательное значение
                        }
                       temperature_3 = rawValue / 10.0; // Масштабирование
                       sys_write_vp(0x2089,(u8*)&temperature_3,2);	
                       rawValue =	0;												
                       } else {
                      
                        }
                    break;							

                  case 0x06:		 
					        // Проверяем длину данных
                    if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                        rawValue = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];
                        if (rawValue & 0x8000) { // Проверяем знак числа
                            rawValue = rawValue - 65536; // Отрицательное значение
                        }
                       temperature_4 = rawValue / 10.0; // Масштабирование
                       sys_write_vp(0x2091,(u8*)&temperature_4,2);	
                       rawValue =	0;												
                       } else {
                      
                        }
                    break;	

                 case 0x07:		 
					       // Проверяем длину данных
                    if (receivedPacket.rcv_dataLength >= 2) {
                        // Извлекаем данные (первый регистр)
                        rawValue = (receivedPacket.rcv_data[0] << 8) | receivedPacket.rcv_data[1];
                        if (rawValue & 0x8000) { // Проверяем знак числа
                            rawValue = rawValue - 65536; // Отрицательное значение
                        }
                       temperature = rawValue / 10.0; // Масштабирование
                       sys_write_vp(0x2005,(u8*)&temperature,2);	
                       rawValue =	0;											
                       } else {
                      
                        }
                    break;
																								
																	
												
					    default:
            break;
			 }
			 
			 
			 
			 
			 }else if (result == 99) {
						sys_write_vp(0x2096, "Lenght\n", 4);
				 sys_delay_ms(20);
				 uart2_rx_sta = 0;
				  uart2_reset(BOUDRATE);
				}else if (result == 98) {
						sys_write_vp(0x2096, "CRC   \n", 4);
					sys_delay_ms(20);
					uart2_rx_sta = 0;
             uart2_reset(BOUDRATE);
				}else {
						sys_write_vp(0x2096, "ERROR\n", 4);
					sys_delay_ms(20);
					uart2_rx_sta = 0;
					 uart2_reset(BOUDRATE);
				}
				
		  	uart2_rx_sta = 0;
			  len=0;
			 
			 for(i=0;i<UART2_PACKET_MAX_LEN;i++)
			{
				uart2_buf[i]=0;
			}
			 		 
			rcv_complete=1;
		}
	 
		
	
		
if (polling_state==0) {
	     if (current_device >= DEVICES) {
           current_device = 0; // Сбрасываем индекс, если он выходит за границы
          }
	
					
		sys_delay_ms(15);
		temp_request = request[current_device];
		sys_write_vp(0x2000,(u8*)&current_device,1);
    command_value = temp_request.command; // Присваивание значения
    sys_write_vp(0x2001, &temp_request.command, 1); // Запись значения команды
		sys_write_vp(0x2002, &temp_request.start_register, 1); // Запись первого регистра
					
		if(command_value==0x03){			
    data_len=(temp_request.num_registers * 2)+5;	}
		
		if(command_value==0x10){			
    data_len=8;}
		
		if(command_value==0x5){			
    data_len=8;}
		
		if(command_value==0x6){			
    data_len=8;}
		
		if(command_value==0x0F){			
    data_len=8;}
		
		sys_write_vp(0x2003,(u16*)&data_len, 2);	
    sys_write_vp(0x2004, &temp_request.address, 1);
		polling_timer=POLLING_TIME; 
		polling_state=1;
		
		
		
		btn_val=0;
		sys_read_vp(0x2079,(u16*)&btn_val,1);
		sys_read_vp(0x2064,(u16*)&pwm_width,1);
		sys_read_vp(0x2073,(u16*)&freq,1);
		
		send_reg[0]=pwm_width;
		send_reg[1]=pwm_width;
		send_reg[2]=pwm_width;
		send_reg[3]=pwm_width;
		send_reg[4]=freq;
		
		coil_1=btn_val&0x2;
		coil_2=btn_val&0x4;
		coil_3=btn_val&0x8;
		coil_4=btn_val&0x10;
		auto_manual=btn_val&0x20;
		pwr_btn=btn_val&0x40;
		
		icon_1=(coil_1>>1)&0x01;
		icon_2=(coil_2>>2)&0x01;
		icon_3=(coil_3>>3)&0x01;
		icon_4=(coil_4>>4)&0x01;
		pwr_icon=(pwr_btn>>6)&0x01;
		sys_write_vp(0x2105,(u16*)&icon_1,1);
		sys_write_vp(0x2107,(u16*)&icon_2,1);
		sys_write_vp(0x2109,(u16*)&icon_3,1);
		sys_write_vp(0x2111,(u16*)&icon_4,1);
		sys_write_vp(0x2113,(u16*)&pwr_icon,1);
		
		if((pwr_icon==1)&&(power_delay==0)){
		request[0].command = 0x05;
		request[0].start_register = 0x0000;
		request[0].num_registers = 0xFF00;
		power_delay=1;
		power_timer=0;
		}
		
		
		if((pwr_icon==1)&&(power_delay==1)&&(power_timer>=POWER_TIME)){
		request[0].command = 0x05;
		request[0].start_register = 0x0001;
		request[0].num_registers = 0xFF00;
		power_delay=1;
		power_timer=0;
		}
	
		if(pwr_icon==0){
		request[0].command = 0x0F;
		request[0].start_register = 0x0000;
		request[0].num_registers = 0x0008;
		request[0].special_cmd = 0x0100;
		power_delay=0;
		power_timer=0;
		}
		
		btn_val&= 0x01;
		setBitInUint16(&send_reg[7], 0, btn_val);
		setBitInUint16(&send_reg[7], 1, coil_1);
		setBitInUint16(&send_reg[7], 2, coil_2);
		setBitInUint16(&send_reg[7], 3, coil_3);
		setBitInUint16(&send_reg[7], 4, coil_4);
		setBitInUint16(&send_reg[7], 5, auto_manual);
		setBitInUint16(&send_reg[7], 6, pwr_btn);
		auto_manual=(auto_manual>>5)&0x01;
		
		sys_write_vp(0x2075,(u16*)&btn_val,1);
		sys_write_vp(0x2060,(u16*)&auto_manual,1);
		
		modbus_requests((ModbusRequest*)&temp_request,send_reg,8);			
		sys_tick=IDLE_TIME;
	     }

      polling_timer--;
		  power_timer++;
		
		// Состояние 1: Ожидание ответа
    if (polling_state == 1) {
        // Если получен ответ
			
        if (rcv_complete==1) {
					  sys_write_vp(0x2042, "Received        \n", 9);			
            // Переход к следующему устройству
            current_device=current_device+1;
            polling_state = 0;  // Возврат в состояние отправки
					  rcv_complete=0;
					  polling_timer=POLLING_TIME;
					  sys_tick=IDLE_TIME;
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
						sys_tick=IDLE_TIME;
        }			
    }	
	}
		
}




