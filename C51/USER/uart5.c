

#include "uart5.h"


#if(UART5_INT_EN)
xdata u16 uart5_rx_sta;//bit15Used to mark whether a complete data packet has been received, bit[14:0] is used to store the length of the current data packetxdata u8
xdata u8  uart5_buf[UART5_PACKET_MAX_LEN+2];//Leave 2 blank characters
xdata u8  uart5_step;

//Serial port 5 interrupt service routine
//When sending data, the interrupt must be turned off, here is only responsible for processing the receiving interrupt.
void uart5_isr() interrupt 13 { // Адрес прерывания для UART5 (0x6B)
    u8 res;

    if (SCON3R & 0x01) { // Проверяем флаг приёма RI3
        SCON3R &= ~0x01; // Сбрасываем флаг приёма
        res = SBUF3_RX;  // Читаем принятые данные из регистра

        if (uart5_rx_sta & UART5_PACKET_OK) {
            // Если ранее принятое сообщение ещё не обработано, выходим
            return;
        }

        if (uart5_step == 0) { // Начинаем обработку принятого пакета
            if (res == '\r') {
                // Если получен символ '\r', переходим к ожиданию '\n'
                uart5_step = 1;
            } else if (res == '\n') {
                // Если получен символ '\n', помечаем пакет как завершённый
                uart5_rx_sta |= UART5_PACKET_OK;
            } else {
                // Сохраняем данные в буфер, если это не конец пакета
                if (uart5_rx_sta >= UART5_PACKET_MAX_LEN) {
                    // Если буфер переполнен, сбрасываем статус и начинаем заново
                    uart5_rx_sta = 0;
                }
                uart5_buf[uart5_rx_sta++] = res; // Сохраняем данные в буфер
            }
        } else if (uart5_step == 1) { // Ожидаем завершающий символ '\n'
            uart5_step = 0;
            if (res == '\n') {
                // Если следующий символ после '\r' — это '\n', помечаем пакет завершённым
                uart5_rx_sta |= UART5_PACKET_OK;
            } else {
                // Если это не '\n', сбрасываем статус и начинаем заново
                uart5_rx_sta = 0;
            }
        }
    }
}

#endif

// Serial port 5 initialization
void uart5_init(u32 baud) {
    // MUX_SEL |= 0x80; // Setting bit6 and bit7 to 1 exports UART5 interface to appropriate pins
    P0MDOUT &= 0x3F; // Clear bits for UART5 pins
    P0MDOUT |= 0x80; // Set P0.7 as output for TX and P0.6 as input for RX

    ADCON = 0x80; // Select SREL3H:L as the baud rate generator for UART5
    SCON3T = 0x80; // Enable transmission
    SCON3R = 0x80; // Enable reception

    // Baud rate setting formula:
    // SREL3H:L = 1024 - (FOSC / (32 * baud rate))
    baud = 1024 - (u16)(3225600.0f / baud);
    BODE3_DIV_H = (baud >> 8) & 0xFF; // High byte of the baud rate divisor
    BODE3_DIV_L = baud & 0xFF;        // Low byte of the baud rate divisor

    #if(UART5_INT_EN)
	
	      IP1 = 0x28; // Биты 3 (G3) и 5 (G5) установлены
        IP0 = 0x00; // Остальные группы остаются с низким приоритетом

        ES3R = 1; // Enable UART5 RX interrupt
        ES3T = 1; // Enable UART5 TX interrupt
        EA = 1;   // Enable global interrupts
    #else
        ES3R = 0; // Disable UART5 RX interrupt
        ES3T = 0; // Disable UART5 TX interrupt
    #endif
}


void u5_send_byte(u8 byte) {
    ES3T = 0;              // Отключаем прерывания передачи для UART5
    SBUF3_TX = byte;       // Отправляем байт через регистр передачи
    while (!SCON3T);          // Ждём завершения передачи (TI3 == 1)
    SCON3T = 0;               // Сбрасываем флаг передачи
    #if(UART5_INT_EN)
        ES3T = 1;          // Повторно включаем прерывания передачи, если активированы
    #endif
}

//Send data
void u5_send_bytes(u8 *bytes, u16 len) {
    u16 i;

    ES3T = 0;              // Отключаем прерывания передачи UART5
    for (i = 0; i < len; i++) {
        SBUF3_TX = bytes[i]; // Записываем текущий байт в регистр передачи
        while (!(SCON3T & 0x01)); // Ждём, пока флаг TI не будет установлен
        SCON3T &= ~0x01;       // Сбрасываем флаг TI
    }
    #if(UART5_INT_EN)
        ES3T = 1;          // Включаем прерывания передачи, если они активированы
    #endif
}


//Implement printf function with uart2 serial port
char putchar5(char c)
{
	u5_send_byte(c);
	
	return c;
}


