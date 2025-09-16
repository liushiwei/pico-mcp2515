#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "lin/lin.h"
#include <string.h>

extern int led_on_time_count;
extern u_int8_t can0CheckBitrateSuccess ;//检测波特率成功标志
extern u_int8_t can1CheckBitrateSuccess ;//检测波特率成功标志
extern u_int8_t linCheckBitrateSuccess ;//检测波特率成功标志
extern int can0_auto_bitrate ; //自动检测的波特率
extern int can1_auto_bitrate ; //自动检测的波特率
extern int lin_auto_bitrate ; //自动检测的波特率
extern u_int8_t can0_message_count ; //有效消息计数器
extern u_int8_t can1_message_count ; // 有效消息计数器

extern int can0_mode ; // 1为正常模式，0为自动检测波特率模式
extern int can1_mode ; // 1为正常模式，0为自动检测波特率模式
extern int lin_mode ; // 1为正常模式，0为自动检测波特率模式
extern u_int8_t can0_check_count ; // 消息计数器
extern u_int8_t can1_check_count ; // 消息计数器
extern u_int8_t lin_check_count ; // 消息计数器
extern int baud_check_len;

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

        sleep_ms(5);

        
    }
};

void core1_entry() {
    while (1) {
        auto_check_baudrate();
        lin_poll_timeout();
        sleep_ms(2);  // 这个核自己延时，不影响主核
    }
}