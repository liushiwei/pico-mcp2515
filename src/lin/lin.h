#ifndef _LIN_H_
#define _LIN_H_
#include <stdio.h>
#include "pico/stdlib.h"
#define LIN_UART_ID uart1
#define LIN_UART_IRQ UART1_IRQ
#define LIN_UART_TX_PIN 8
#define LIN_UART_RX_PIN 9

extern uint8_t baud_check_buf[64];
extern int baud_check_len;
extern uint32_t lin_common_baud[];
enum
{
	LIN_STATE_BREAK,
	LIN_STATE_SYNC,
	LIN_STATE_PID,	
	LIN_DATA,
    LIN_CHECKSUM
};
enum LIN_SPEED {
    LIN_20KBPS,
    LIN_19K2BPS,
    LIN_18KBPS,
    LIN_17KBPS,
    LIN_16KBPS,
    LIN_15KBPS,
    LIN_14KBPS,
    LIN_13KBPS,
    LIN_12KBPS,
    LIN_11KBPS,
    LIN_10K4BPS,
    LIN_10KBPS,
    LIN_9K6BPS,
    LIN_9KBPS,
    LIN_8KBPS,
    LIN_7KBPS,
    LIN_6KBPS,
    LIN_5KBPS,
    LIN_4KBPS,
    LIN_3KBPS,
    LIN_2K4BPS,
    LIN_2KBPS,
    LIN_1K2BPS,
    LIN_1KBPS
};
void lin_init(int baudrate);
uint8_t lin_calc_pid(uint8_t id);
//uint8_t lin_checksum(uint8_t *data, int len, uint8_t pid, bool enhanced);
uint8_t is_lin_baudrate_ok(void);
#endif