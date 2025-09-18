/**
 * 用can工具加120欧电阻
 * 发送数据格式
 * AB,01,01,AB01020304050607
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <iostream>
#include <bitset>
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "hardware/sync.h"
#include "lin/lin.h"
#include "pico/multicore.h"
#include "core1.h"
#include "pico-mcp2515.h"

#define RESET_PIN 21 // 板子烧录开关
absolute_time_t  press_start ;
#define LONG_PRESS_MS 1000  // 长按阈值：2 秒
__attribute__((section(".data.ramfunc"))) void jump_to_usb_boot() {
    reset_usb_boot(0, 0);  // 进入 USB BOOT 模式
}

#define TX_SPINLOCK_ID 0  // 使用硬件 spinlock 0
//#define SEND_CAN_EMBEDDED_DATA  // 发送嵌入式数据
#define AUTO_CHECK_BITRATE
#define LED_ON_TIME 1000
int led_on_time_count = LED_ON_TIME; //LED亮的时间计数器
u_int8_t can0CheckBitrateSuccess = 0;//检测波特率成功标志
u_int8_t can1CheckBitrateSuccess = 0;//检测波特率成功标志
u_int8_t linCheckBitrateSuccess = 0;//检测波特率成功标志
int can0_auto_bitrate = 0; //自动检测的波特率
int can1_auto_bitrate = 0; //自动检测的波特率
int lin_auto_bitrate = 0; //自动检测的波特率
u_int8_t can0_message_count = 0; //有效消息计数器
u_int8_t can1_message_count = 0; // 有效消息计数器

int can0_mode = 0; // 1为正常模式，0为自动检测波特率模式
int can1_mode = 0; // 1为正常模式，0为自动检测波特率模式
int lin_mode = 0; // 1为正常模式，0为自动检测波特率模式
u_int8_t can0_check_count = 200; // 消息计数器
u_int8_t can1_check_count = 200; // 消息计数器
u_int8_t lin_check_count = 200; // 消息计数器
uint8_t lin_head[2];
uint8_t lin_head_index = 0;

#define CAN1_INT_PIN 2 // CAN1中断引脚
MCP2515 can0; //can1 接受
//struct can_frame rx;
struct can_frame canMsg1;

#define CAN2_INT_PIN 3 // CAN2中断引脚

MCP2515 can1; //can2 接受
//struct can_frame rx;
struct can_frame canMsg2;



const char * SEPARATOR = ",";
const char *  TERMINATOR = "\n";

uint8_t state = PKG_STATE_STRART;




const uint8_t SIDL_EXTENDED_MSGID = 1U << 3U;

const char *SPEED_STR[] = {    
    "CAN_5KBPS",
    "CAN_10KBPS",
    "CAN_20KBPS",
    "CAN_31K25BPS",
    "CAN_33KBPS",
    "CAN_40KBPS",
    "CAN_50KBPS",
    "CAN_80KBPS",
    "CAN_83K3BPS",
    "CAN_95KBPS",
    "CAN_100KBPS",
    "CAN_125KBPS",
    "CAN_200KBPS",
    "CAN_250KBPS",
    "CAN_500KBPS",
    "CAN_1000KBPS"
};
#ifdef SEND_CAN_EMBEDDED_DATA
// 使用自动生成的符号名
extern const char _binary_data_txt_start[];
extern const char _binary_data_txt_end[];

// 文本读取器结构
typedef struct {
    const char* start;
    const char* end;
    const char* current;
    size_t buffer_size;
    char* line_buffer;
} TextReader;

// 初始化文本读取器
void text_reader_init(TextReader* reader, 
                     const char* data_start, 
                     const char* data_end,
                     char* buffer, 
                     size_t buffer_size) {
    reader->start = data_start;
    reader->end = data_end;
    reader->current = data_start;
    reader->line_buffer = buffer;
    reader->buffer_size = buffer_size;
}

// 读取下一行
bool text_reader_next_line(TextReader* reader) {
    if (reader->current >= reader->end) {
        return false;
    }
    
    const char* line_start = reader->current;
    const char* pos = reader->current;
    size_t line_length = 0;
    
    // 查找行结束
    while (pos < reader->end && *pos != '\n' && *pos != '\r') {
        pos++;
        line_length++;
    }
    
    // 复制行内容（带截断保护）
    size_t copy_size = line_length;
    if (copy_size >= reader->buffer_size) {
        copy_size = reader->buffer_size - 1;
    }
    
    memcpy(reader->line_buffer, line_start, copy_size);
    reader->line_buffer[copy_size] = '\0';
    
    // 跳过换行符
    while (pos < reader->end && (*pos == '\n' || *pos == '\r')) {
        pos++;
    }
    
    reader->current = pos;
    return true;
}
#endif // SEND_CAN_EMBEDDED_DATA

uint8_t hex_string_to_bytes(const char *hex_str) {
     uint8_t high_val = (hex_str[0] <= '9') ? (hex_str[0] - '0') : 
                          (hex_str[0] >= 'a') ? (hex_str[0] - 'a' + 10) : (hex_str[0] - 'A' + 10);
        
        // 转换低4位
        uint8_t low_val = (hex_str[1] <= '9') ? (hex_str[1] - '0') : 
                         (hex_str[1] >= 'a') ? (hex_str[1] - 'a' + 10) : (hex_str[1] - 'A' + 10);
        
        // 检查非法字符（查表中未显式初始化的位置为0）
        if (high_val == 0 && hex_str[0] != '0') return -1;
        if (low_val == 0 && hex_str[1] != '0') return -1;

        return (high_val << 4) | low_val;
}

int hex_string_to_bytes_opt(const char *hex_str, uint8_t *bytes, size_t bytes_len) {
      size_t hex_len = strlen(hex_str);
    if (hex_len % 2 != 0 || bytes_len < hex_len / 2) 
        return -1;

    for (size_t i = 0; i < hex_len; i += 2) {
        char c1 = hex_str[i];
        char c2 = hex_str[i + 1];
        
        // 快速验证字符范围
        if (!isxdigit(c1) || !isxdigit(c2)) 
            return -1;
        
        // 转换高4位
        uint8_t high_val = (c1 <= '9') ? (c1 - '0') : 
                          (c1 >= 'a') ? (c1 - 'a' + 10) : (c1 - 'A' + 10);
        
        // 转换低4位
        uint8_t low_val = (c2 <= '9') ? (c2 - '0') : 
                         (c2 >= 'a') ? (c2 - 'a' + 10) : (c2 - 'A' + 10);
        
        bytes[i / 2] = (high_val << 4) | low_val;
    }
    return 0;
}

uint8_t hostDataArray[50];
uint8_t hostDataHEX[2];
uint8_t hostDataIndex = 0;
// 函数：host_packet_analyze
// 功能：分析上位机数据包
// 参数：data，数据包中的数据
void host_packet_analyze(uint8_t data)
{
  // 打印数据
  //printf("%c", data);
  switch (state)
  {
  case PKG_STATE_STRART:
    /* code */
    if(data != '\n'){
        hostDataIndex = 0;
        hostDataArray[hostDataIndex]  = data;
        hostDataIndex++;
        state = PKG_STATE_ID;
    }
    break;
  case PKG_STATE_ID:
    if(data == ','){
        state = PKG_STATE_RTR;
        hostDataArray[hostDataIndex] = '\0';
        //printf("can_id %s \n", hostDataArray);
        canMsg1.can_id = strtol((const char *)hostDataArray, NULL, 16); 
        //printf("can_id %lX \n", canMsg1.can_id);
        hostDataIndex = 0;
    }
    else
    {
        hostDataArray[hostDataIndex]  = data;
        hostDataIndex++;
    }
    break;
  case PKG_STATE_RTR:
    if(data == ','){
        state = PKG_STATE_IDE;
        hostDataArray[hostDataIndex] = '\0';
        uint8_t rtr = hex_string_to_bytes((const char *)hostDataArray); 
        if(rtr>0)
        canMsg1.can_id = canMsg1.can_id | CAN_RTR_FLAG;

        hostDataIndex = 0;
    }
    else
    {
        hostDataArray[hostDataIndex]  = data;
        hostDataIndex++;
    }
    break;
  case PKG_STATE_IDE:
    if(data == ','){
        state = PKG_STATE_DATA;
        hostDataArray[hostDataIndex] = '\0';
        uint8_t ide = hex_string_to_bytes((const char *)hostDataArray); 
        if(ide>0)
        canMsg1.can_id = canMsg1.can_id | CAN_EFF_FLAG;

        hostDataIndex = 0;
    }
    else
    {
        hostDataArray[hostDataIndex]  = data;
        hostDataIndex++;
    }
    break;
  case PKG_STATE_DATA:
    if(data == '\n'){
        state = PKG_STATE_STRART;
        for(int i = 0; i < 20; i++) {
            canMsg1.data[i] = 0; 
        }
        hex_string_to_bytes_opt((const char *)hostDataArray, canMsg1.data, hostDataIndex);
        canMsg1.can_dlc = hostDataIndex/2;
        can0.sendMessage(&canMsg1);
        hostDataIndex = 0;
    }else{
        hostDataArray[hostDataIndex]  = data;
        hostDataIndex++;
    }
  default:
    break;
  }
  
}
#if 1
int hex_upper_min2_to_buf(unsigned long v, char *buf) {
    static const char hex[] = "0123456789ABCDEF";
    if (v == 0) {                // special-case 0 -> "00"
        buf[0] = '0';
        buf[1] = '0';
        return 2;
    }

    // 统计需要的十六进制位数（每位为一个 nibble）
    int nibbles = 0;
    unsigned long tmp = v;
    while (tmp) {
        tmp >>= 4;
        ++nibbles;
    }
    if (nibbles < 2) nibbles = 2; // 至少两位

    // 从最高 nibble 到最低 nibble 填充
    for (int i = nibbles - 1, pos = 0; i >= 0; --i, ++pos) {
        buf[pos] = hex[(v >> (i * 4)) & 0xF];
    }

    return nibbles;
}

#endif 
void printHex(long num) {
  //if ( num < 0x10 ){ Serial.print("0"); }
  //printf("%02X", num);
  char buf[32]; // 足够大（max 16 for 64-bit）
  int len = hex_upper_min2_to_buf(num, buf);
  buf[len] = '\0'; // 添加字符串终止符
  printf(buf);
}
void printPacket(int channel ,packet_t * packet) {
    uint32_t save = spin_lock_blocking(spin_lock_instance(TX_SPINLOCK_ID));
    // packet format (hex string): [ID],[RTR],[IDE],[DATABYTES 0..8B]\n
    // example: 014A,00,00,1A002B003C004D\n
    if(channel == 0)
        printf("A");
    else if(channel == 1)
        printf("B");
    else
        printf("L");
    printHex(packet->id);
    printf(SEPARATOR);
    printHex(packet->rtr);
    printf(SEPARATOR);
    printHex(packet->ide);
    printf(SEPARATOR);
    // DLC is determinded by number of data bytes, format: [00]
    for (int i = 0; i < packet->dlc; i++) {
        printHex(packet->dataArray[i]);
    }
    printf(TERMINATOR);
    spin_unlock(spin_lock_instance(TX_SPINLOCK_ID), save);
}


void spi_transmit(uint8_t *tx, uint8_t* rx, size_t len) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    if(rx) {
        spi_write_read_blocking(spi_default, tx, rx, len);
    } else {
        spi_write_blocking(spi_default, tx, len);
    }
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}
void autoCheckBaudrate() {
    while (true)
    {
        //int bitrate = CAN_1000KBPS;
        for (int i = CAN_1000KBPS; i > -1; i--) {
            can0_message_count = 0;
            if(can0.reset() == MCP2515::ERROR_OK){
                printf("can0.reset OK \n");
            }
            sleep_ms(10);
            can0.setBitrate((CAN_SPEED)i, MCP_8MHZ);
            printf("Set bitrate : %s\n", SPEED_STR[i]);
            if(can0.setListenOnlyMode() == MCP2515::ERROR_OK){
                printf("Set listen only mode \n");
                can0_mode = 0 ;
            }else{
                printf("Set listen only mode error \n");
            }
            //can_mode = 0 ;
            
            int j = 200;
            while(j)
            {
                    
                if(can0_message_count > 100){
                    printf("Auto baudrate found: %d\n", can0_message_count);
                    can0CheckBitrateSuccess = true;
                    can0_auto_bitrate = i;
                    return;
                }
                sleep_ms(10);
                j--;    
            }
            
        }
    }
    
  
}

void can1_interrupts(uint gpio, uint32_t event) {
    uint8_t status = can0.getInterrupts();
    if(status&0x03 && !(status&0x80)){

        status = can0.getStatus();
        if(status&0x01)
		{
			status &= 0xfe;
			uint8_t tx[14] = {
            0x90,
            };
            uint8_t rx[sizeof(tx)] = {0};
            spi_transmit(tx, rx, sizeof(tx));
            //if(can0.checkError()){
            //    printf("CANINTF_RX0IF  recive Error \n");
            //}
            can0.modifyRegister(MCP2515::REGISTER::MCP_CANINTF, MCP2515::CANINTF::CANINTF_RX0IF, 0);
            packet_t txPacket;
            txPacket.dlc = rx[5] & 0b1111;
            if(rx[2] & SIDL_EXTENDED_MSGID) {
                txPacket.id =  (rx[1] << 21U)
                    | ((rx[2] >> 5U) << 18U)
                    | ((rx[2] & 0b11) << 16U)
                    | (rx[3] << 8U)
                    | rx[4]
                    | (1 << 31U); // extended frame, see linux/can.h
            } else {
                txPacket.id =  (rx[1] << 3U) | (rx[2] >> 5U);
            }
            txPacket.ide = (txPacket.id&CAN_EFF_FLAG) > 0 ? 1 : 0;
            txPacket.rtr = (txPacket.id&CAN_RTR_FLAG) > 0 ? 1 : 0;
            memcpy(txPacket.dataArray, &rx[6], txPacket.dlc);
            can0_message_count++;
            //printf("== can0_message_count ++ \n");
            if(can0CheckBitrateSuccess)
            printPacket(0,&txPacket);
            led_on_time_count = LED_ON_TIME;
		}
		else if(status&0x02)
		{
			status &= 0xfd;
			uint8_t tx[14] = {
            0x94,
            };
            uint8_t rx[sizeof(tx)] = {0};
            spi_transmit(tx, rx, sizeof(tx));
            //if(can0.checkError()){
            //    printf("CANINTF_RX1IF recive Error \n");
            //}
            can0.modifyRegister(MCP2515::REGISTER::MCP_CANINTF, MCP2515::CANINTF::CANINTF_RX1IF, 0);
            packet_t txPacket;
            txPacket.dlc = rx[5] & 0b1111;
            if(rx[2] & SIDL_EXTENDED_MSGID) {
                txPacket.id =  (rx[1] << 21U)
                    | ((rx[2] >> 5U) << 18U)
                    | ((rx[2] & 0b11) << 16U)
                    | (rx[3] << 8U)
                    | rx[4]
                    | (1 << 31U); // extended frame, see linux/can.h
            } else {
                txPacket.id =  (rx[1] << 3U) | (rx[2] >> 5U);
            }
            txPacket.ide = (txPacket.id&CAN_EFF_FLAG) > 0 ? 1 : 0;
            txPacket.rtr = (txPacket.id&CAN_RTR_FLAG) > 0 ? 1 : 0;
            memcpy(txPacket.dataArray, &rx[6], txPacket.dlc);
            can0_message_count++;
            //printf("== can0_message_count ++ \n");
            if(can0CheckBitrateSuccess)
            printPacket(0,&txPacket);
            led_on_time_count = LED_ON_TIME;
		}
		else;
        
        if((can0_message_count > 20) && (can0_mode == 0) ){
            if(can0.setNormalMode()== MCP2515::ERROR_OK){
                //printf("== setNormalMode OK \n");
                //can0_mode = 1 ;
            }else{
                can0.setNormalMode();
                printf("!SET_NORMALMODE_ERROR \n");
            }
        }
        #if 0
        printf("CAN RX0 Interrupt\n");
        if(can0.readMessage(&rx) == MCP2515::ERROR_OK) {
            //printf("New frame from ID: %10x  %10x   %10x  %10x \n", rx.can_id,rx.can_id&CAN_ERR_MASK,rx.can_id&CAN_EFF_FLAG,rx.can_id&CAN_RTR_FLAG);
            
            packet_t txPacket;
            txPacket.id = rx.can_id&CAN_ERR_MASK;
            txPacket.ide = (rx.can_id&CAN_EFF_FLAG) > 0 ? 1 : 0;
            txPacket.rtr = (rx.can_id&CAN_RTR_FLAG) > 0 ? 1 : 0;
            txPacket.dlc = rx.can_dlc;  
            for (int i = 0; i < rx.can_dlc; i++) {
                    txPacket.dataArray[i] = rx.data[i];
                }
            can0_message_count++;
            printPacket(&txPacket);
            led_on_time_count = LED_ON_TIME;
        }
        #endif
        

    }
    
	can0.clearInterrupts();
}

void can2_interrupts(uint gpio, uint32_t event) {
    uint8_t status = can1.getInterrupts();
    if(status&0x03 && !(status&0x80)){

        status = can1.getStatus();
        if(status&0x01)
		{
			status &= 0xfe;
			uint8_t tx[14] = {
            0x90,
            };
            uint8_t rx[sizeof(tx)] = {0};
            spi_transmit(tx, rx, sizeof(tx));
            //if(can1.checkError()){
            //    printf("CANINTF_RX0IF  recive Error \n");
            //}
            can1.modifyRegister(MCP2515::REGISTER::MCP_CANINTF, MCP2515::CANINTF::CANINTF_RX0IF, 0);
            packet_t txPacket;
            txPacket.dlc = rx[5] & 0b1111;
            if(rx[2] & SIDL_EXTENDED_MSGID) {
                txPacket.id =  (rx[1] << 21U)
                    | ((rx[2] >> 5U) << 18U)
                    | ((rx[2] & 0b11) << 16U)
                    | (rx[3] << 8U)
                    | rx[4]
                    | (1 << 31U); // extended frame, see linux/can.h
            } else {
                txPacket.id =  (rx[1] << 3U) | (rx[2] >> 5U);
            }
            txPacket.ide = (txPacket.id&CAN_EFF_FLAG) > 0 ? 1 : 0;
            txPacket.rtr = (txPacket.id&CAN_RTR_FLAG) > 0 ? 1 : 0;
            memcpy(txPacket.dataArray, &rx[6], txPacket.dlc);
            can1_message_count++;
            //printf("== can1_message_count ++ \n");
            if(can1CheckBitrateSuccess)
            printPacket(1,&txPacket);
            led_on_time_count = LED_ON_TIME;
		}
		else if(status&0x02)
		{
			status &= 0xfd;
			uint8_t tx[14] = {
            0x94,
            };
            uint8_t rx[sizeof(tx)] = {0};
            spi_transmit(tx, rx, sizeof(tx));
            //if(can1.checkError()){
            //    printf("CANINTF_RX1IF recive Error \n");
            //}
            can1.modifyRegister(MCP2515::REGISTER::MCP_CANINTF, MCP2515::CANINTF::CANINTF_RX1IF, 0);
            packet_t txPacket;
            txPacket.dlc = rx[5] & 0b1111;
            if(rx[2] & SIDL_EXTENDED_MSGID) {
                txPacket.id =  (rx[1] << 21U)
                    | ((rx[2] >> 5U) << 18U)
                    | ((rx[2] & 0b11) << 16U)
                    | (rx[3] << 8U)
                    | rx[4]
                    | (1 << 31U); // extended frame, see linux/can.h
            } else {
                txPacket.id =  (rx[1] << 3U) | (rx[2] >> 5U);
            }
            txPacket.ide = (txPacket.id&CAN_EFF_FLAG) > 0 ? 1 : 0;
            txPacket.rtr = (txPacket.id&CAN_RTR_FLAG) > 0 ? 1 : 0;
            memcpy(txPacket.dataArray, &rx[6], txPacket.dlc);
            can1_message_count++;
            //printf("== can1_message_count ++ \n");
            if(can1CheckBitrateSuccess)
            printPacket(1,&txPacket);
            led_on_time_count = LED_ON_TIME;
		}
		else;
        
        if((can1_message_count > 5) && (can1_mode == 0) ){
            if(can1.setNormalMode()== MCP2515::ERROR_OK){
                //printf("== setNormalMode OK \n");
                //can1_mode = 1 ;
            }else{
                can1.setNormalMode();
                printf("!SET_NORMALMODE_ERROR \n");
            }
        }
        #if 0
        printf("CAN RX0 Interrupt\n");
        if(can1.readMessage(&rx) == MCP2515::ERROR_OK) {
            //printf("New frame from ID: %10x  %10x   %10x  %10x \n", rx.can_id,rx.can_id&CAN_ERR_MASK,rx.can_id&CAN_EFF_FLAG,rx.can_id&CAN_RTR_FLAG);
            
            packet_t txPacket;
            txPacket.id = rx.can_id&CAN_ERR_MASK;
            txPacket.ide = (rx.can_id&CAN_EFF_FLAG) > 0 ? 1 : 0;
            txPacket.rtr = (rx.can_id&CAN_RTR_FLAG) > 0 ? 1 : 0;
            txPacket.dlc = rx.can_dlc;  
            for (int i = 0; i < rx.can_dlc; i++) {
                    txPacket.dataArray[i] = rx.data[i];
                }
            can0_message_count++;
            printPacket(&txPacket);
            led_on_time_count = LED_ON_TIME;
        }
        #endif
        

    }
    
	can1.clearInterrupts();
}
void all_interrupts(uint gpio, uint32_t event) {
    #if 0
    std::cout   << time_us_64()
                << " IRQ: gpio "
                << gpio
                << " event "
                << std::bitset<32>(event)
                << std::endl;
    #endif
   
    if(gpio == CAN1_INT_PIN) {
        //printf("CAN1_INT_PIN, \n");
        can1_interrupts(gpio, event);
    }else  if(gpio == CAN2_INT_PIN) {
        //printf("CAN2_INT_PIN, \n");
        can2_interrupts(gpio, event);
    }
    if(gpio == RESET_PIN && (event & GPIO_IRQ_EDGE_FALL)) // 如果 GPIO 21 上升沿触发
    {
        press_start = get_absolute_time();
        printf("Reset pin LOW, \n");
    }
    if(gpio == RESET_PIN && (event & GPIO_IRQ_EDGE_RISE)) // 如果 GPIO 21 上升沿触发
    {
        uint32_t held_time =  absolute_time_diff_us(press_start, get_absolute_time()) / 1000;
        if (held_time < LONG_PRESS_MS) {        // 短按重启
            printf("Reset pin short press reboot .\n", held_time);
            watchdog_reboot(0, 0, 0); // 软件重启
        }else{                      // 长按进入烧录模式
            printf("Reset pin LONG_PRESS_MS , jump_to_usb_boot.\n");
            jump_to_usb_boot();
        }
    }
    
    
				
}

#if 0
void auto_check_baudrate() {
    #if 0
    if(can0_mode == 0&&(!can0CheckBitrateSuccess)){ //自动检测波特率模式
        if(can0_check_count == 0){
            led_on_time_count = 10;
            can0_check_count = 200;
            if( can0_auto_bitrate == CAN_5KBPS ) {
            can0_auto_bitrate = CAN_1000KBPS;
            }else{
                can0_auto_bitrate--;
            }
        }
        
        if(can0_check_count == 200){ //第一次设置该波特率
            can0_message_count = 0;
            if(can0.reset() == MCP2515::ERROR_OK){
                printf("!CAN0_RESET_OK \n");
            }
            sleep_ms(10);
            can0.setBitrate((CAN_SPEED)can0_auto_bitrate, MCP_8MHZ);
            printf("!CAN0_CHECK_BITRATE : %s\n", SPEED_STR[can0_auto_bitrate]);
            if(can0.setListenOnlyMode() == MCP2515::ERROR_OK){
                //printf("Set listen only mode \n");
                can0_mode = 0 ;
            }else{
                printf("!CAN0_SET_LISTER_MODE_ERROR \n");
            }
        }
        can0_check_count--;
        if(can0_message_count > 100){
            //printf("Auto baudrate found: %d\n", can0_message_count);
            can0CheckBitrateSuccess = true;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            printf("!CAN0_OK_SET_BITRATE:%s\n", SPEED_STR[can0_auto_bitrate]);
            //can0_auto_bitrate = i;
            //return;
        }
        sleep_ms(10);

        
    }
    
    if(can1_mode == 0&&(!can1CheckBitrateSuccess)){ //自动检测波特率模式
        if(can1_check_count == 0){
            led_on_time_count = 10;
            can1_check_count = 200;
            if( can1_auto_bitrate == CAN_5KBPS ) {
            can1_auto_bitrate = CAN_1000KBPS;
            }else{
                can1_auto_bitrate--;
            }
        }
        
        if(can1_check_count == 200){ //第一次设置该波特率
            can1_message_count = 0;
            if(can1.reset() == MCP2515::ERROR_OK){
                printf("!CAN1_RESET_OK \n");
            }
            sleep_ms(10);
            can1.setBitrate((CAN_SPEED)can1_auto_bitrate, MCP_8MHZ);
            printf("!CAN1_CHECK_BITRATE : %s\n", SPEED_STR[can1_auto_bitrate]);
            if(can1.setListenOnlyMode() == MCP2515::ERROR_OK){
                //printf("Set listen only mode \n");
                can1_mode = 0 ;
            }else{
                printf("!CAN0_SET_LISTER_MODE_ERROR \n");
            }
        }
        can1_check_count--;
        if(can1_message_count > 100){
            //printf("Auto baudrate found: %d\n", can1_message_count);
            can1CheckBitrateSuccess = true;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            printf("!CAN1_OK_SET_BITRATE:%s\n", SPEED_STR[can0_auto_bitrate]);
            //can0_auto_bitrate = i;
            //return;
        }
        sleep_ms(10);

        
    }
    #endif
    if(lin_mode == 0&&(!linCheckBitrateSuccess)){ //自动检测波特率模式
        if(lin_check_count == 0){
            led_on_time_count = 10;
            lin_check_count = 200;
            baud_check_len = 0;
            memset(baud_check_buf, 0, sizeof(baud_check_buf));
            if( lin_auto_bitrate == LIN_20KBPS ) {
                lin_auto_bitrate = LIN_1KBPS;
            }else{
                lin_auto_bitrate--;
            }
        }
        if(lin_check_count == 200){ //第一次设置该波特率
            lin_init(lin_common_baud[lin_auto_bitrate]);
            sleep_ms(50); // 给总线稳定时间
        }
        lin_check_count--;

        
        if(is_lin_baudrate_ok()){
            linCheckBitrateSuccess = true;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            printf("!LIN_OK_SET_BITRATE:%u\n", lin_common_baud[lin_auto_bitrate]);
            
        }

        sleep_ms(10);

        
    }
};
#endif
int main() {
    stdio_init_all();
    multicore_launch_core1(core1_entry);
    can0CheckBitrateSuccess = 0;
    #ifndef PICO_DEFAULT_LED_PIN
    #warning blink example requires a board with a regular LED
    #else
    //const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    #endif
    //设置复位按键中断回调函数
    gpio_put(RESET_PIN, 1);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    gpio_set_irq_enabled_with_callback(RESET_PIN, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true, &all_interrupts);  

    //设置CAN1中断回调函数
    gpio_init(CAN1_INT_PIN);
    gpio_set_dir(CAN1_INT_PIN, GPIO_IN);

    gpio_pull_up(CAN1_INT_PIN);
    gpio_set_irq_enabled_with_callback(
            CAN1_INT_PIN,
            GPIO_IRQ_EDGE_FALL,
            true,
            &all_interrupts);

    //设置CAN2中断回调函数
    gpio_init(CAN2_INT_PIN);
    gpio_set_dir(CAN2_INT_PIN, GPIO_IN);

    gpio_pull_up(CAN2_INT_PIN);
    gpio_set_irq_enabled_with_callback(
            CAN2_INT_PIN,
            GPIO_IRQ_EDGE_FALL,
            true,
            &all_interrupts);


    #ifdef AUTO_CHECK_BITRATE
    //autoCheckBaudrate() ;
    #else
    can0CheckBitrateSuccess = true;
    can0_auto_bitrate = CAN_500KBPS;

    can0.reset();
    can0.setBitrate(CAN_500KBPS, MCP_8MHZ);
    can0.setNormalMode();
    #endif
    #if 0
    if(!can0CheckBitrateSuccess){
        can0.reset();
        can0.setBitrate(CAN_500KBPS, MCP_8MHZ);
        can0.setNormalMode();
        printf("!AUTO_CHECK_FAILED_SET_BITRATE:%s\n", SPEED_STR[CAN_500KBPS]);
    }else{
        gpio_put(LED_PIN, 1);
        //sleep_ms(3000);
        //can0.setNormalMode();
        printf("!OK_SET_BITRATE:%s\n", SPEED_STR[can0_auto_bitrate]);
    }
    #endif
    #ifdef SEND_CAN_EMBEDDED_DATA
    //Listen loop
    char line_buffer[128];
    while(true) {
    TextReader reader;
    text_reader_init(&reader, 
                    _binary_data_txt_start, 
                    _binary_data_txt_end,
                    line_buffer, 
                    sizeof(line_buffer));
    
    int line_num = 1;
    
    while (text_reader_next_line(&reader)) {
        printf("Line %d: %s\n", line_num, line_buffer);
        line_buffer[4]='\0';
        canMsg1.can_id = strtol((const char *)line_buffer, NULL, 16); 
        canMsg1.can_dlc = 8;
        //printf("line_buffer %s \n",line_buffer+5); 
        memcpy(hostDataArray,line_buffer+5, 16);
        //printf("hostDataArray %s \n",hostDataArray); 
        hex_string_to_bytes_opt((const char *)hostDataArray, canMsg1.data, 20);
        //printf("canMsg1.can_id %lX \n", canMsg1.can_id);
        can0.sendMessage(&canMsg1);
        sleep_ms(100);
        line_num++;
    }
    }
    #else
    while(true) {
        
        int ch = getchar_timeout_us(0);  // 非阻塞获取输入字符
        //printf("led_on_time_count %d \n",led_on_time_count);
        if (ch != PICO_ERROR_TIMEOUT) {
            host_packet_analyze(ch);
        }
        
        if(led_on_time_count > 0){
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            led_on_time_count--;
        }else{
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
        //auto_check_baudrate();
        #if 0
        packet_t txPacket;
        txPacket.dlc = 8;
           
        txPacket.id = 0x811; // extended frame, see linux/can.h
          
        txPacket.ide = 0;
        txPacket.rtr = 0;
        memset(txPacket.dataArray, 8, 8);
        printPacket(0,&txPacket);
        #endif
        //printf("HELLO WORLD ----------------------\n");
        //can0.sendMessage(&canMsg1);
        //delay_ms(100);
        //printf("Send New frame from ID: %10x\n",  canMsg1.can_id);
        //gpio_put(LED_PIN, 1);
        //sleep_ms(250);
        //gpio_put(LED_PIN, 0);
        //sleep_ms(250);
    }
    #endif

    return 0;
}
