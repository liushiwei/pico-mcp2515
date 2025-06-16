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

//#define SEND_CAN_EMBEDDED_DATA
#define AUTO_CHECK_BITRATE
#define LED_ON_TIME 10000
int led_on_time_count = LED_ON_TIME;
u_int8_t checkBitrateSuccess = 0;
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
MCP2515 can0;
struct can_frame rx;
struct can_frame canMsg1;
const char * SEPARATOR = ",";
const char *  TERMINATOR = "\n";
enum
{
	PKG_STATE_STRART,
    PKG_STATE_ID,
    PKG_STATE_RTR,
    PKG_STATE_IDE,
    PKG_STATE_DATA,
};
uint8_t state = PKG_STATE_STRART;


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

typedef struct {
  long id;
  uint8_t rtr;
  uint8_t ide;
  uint8_t dlc;
  uint8_t dataArray[20];
} packet_t;
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
void printHex(long num) {
  //if ( num < 0x10 ){ Serial.print("0"); }
  printf("%02X", num);
}
void printPacket(packet_t * packet) {
  // packet format (hex string): [ID],[RTR],[IDE],[DATABYTES 0..8B]\n
  // example: 014A,00,00,1A002B003C004D\n
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
}

void autoCheckBaudrate() {
    while (true)
    {
        int bitrate = CAN_1000KBPS;
        u_int8_t message_count = 0;
        for (int i = CAN_1000KBPS; i > -1; i--) {
            can0.reset();
            can0.setBitrate((CAN_SPEED)i, MCP_8MHZ);
            can0.setListenOnlyMode();
            printf("Set bitrate : %d\n", i);
            int j = 100;
            while(j)
            {
                if (can0.readMessage(&rx) == MCP2515::ERROR_OK) {
                    printf("Auto baudrate found: %d\n", message_count);
                    message_count++;
                    if(message_count > 3){
                        checkBitrateSuccess = true;
                        return;
                    }

                }
                sleep_ms(10);
                j--;    
            }
            
        }
    }
    
  
}
int main() {
    stdio_init_all();
    #ifdef AUTO_CHECK_BITRATE
    autoCheckBaudrate() ;
    #else
    checkBitrateSuccess = true;
    can0.reset();
    can0.setBitrate(CAN_500KBPS, MCP_8MHZ);
    can0.setNormalMode();
    #endif
    #ifndef PICO_DEFAULT_LED_PIN
    #warning blink example requires a board with a regular LED
    #else
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    //gpio_put(LED_PIN, 1);
    //while (true) {
     //   gpio_put(LED_PIN, 1);
     //   sleep_ms(250);
     //   gpio_put(LED_PIN, 0);
     //   sleep_ms(250);
    //}
    #endif
    if(!checkBitrateSuccess){
        can0.reset();
        can0.setBitrate(CAN_500KBPS, MCP_8MHZ);
        can0.setNormalMode();
        printf("Auto baudrate check failed, set default bitrate : %d\n", CAN_500KBPS);
    }else{
        gpio_put(LED_PIN, 1);
        sleep_ms(3000);
        can0.setNormalMode();
        printf("Auto baudrate check success, set default bitrate : %d\n", CAN_500KBPS);
    }
    
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
            printPacket(&txPacket);
            led_on_time_count = LED_ON_TIME;
        }
        int ch = getchar_timeout_us(0);  // 非阻塞获取输入字符
        //printf("led_on_time_count %d \n",led_on_time_count);
        if (ch != PICO_ERROR_TIMEOUT) {
            host_packet_analyze(ch);
        }
        
        if(led_on_time_count > 0){
            gpio_put(LED_PIN, 1);
            led_on_time_count--;
        }else{
            gpio_put(LED_PIN, 0);
        }
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
