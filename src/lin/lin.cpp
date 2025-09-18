#include "lin.h"
#include <inttypes.h>
#include "pico-mcp2515.h"
#include <string.h>

int baud_check_len = 0;
uint32_t lin_common_baud[] = {20000,19200,18000,17000,16000,15000,14000,13000,12000,11000, 
                                10400,10000,9600, 9000,8000,7000,6000,5000,4000,3000,2400,2000,1200,1000};
uint8_t baud_check_buf[64];
u_int8_t lin_message_count = 0; // 有效消息计数器
//uint8_t lin_state = LIN_STATE_BREAK;
extern u_int8_t linCheckBitrateSuccess ;//检测波特率成功标志

//extern struct packet_t;
//extern void printPacket(int channel ,packet_t * packet);


//测试数据 7位 ID 0x43 00 11 22 33 FF CC 52
// 同步阶段最多忽略的 0x00 数量（容忍 Break 残影）
#define MAX_SYNC_SKIP 3

typedef enum {
    LIN_WAIT_BREAK,
    LIN_WAIT_SYNC,
    LIN_RECV_PID,
    LIN_RECV_DATA,
    LIN_RECV_CHECKSUM
} lin_state_t;

static volatile lin_state_t lin_state = LIN_WAIT_BREAK;
static volatile bool break_seen = false;
static uint8_t sync_skip_cnt = 0;

// 帧缓冲
static uint8_t frame_buf[8];
static int frame_len = 0;
static int frame_pos = 0;
static uint8_t current_pid = 0;
uint8_t prev_data = 0;

long lin_time_bytes_us(int bytes, int baud);
uint32_t lin_flash_timeout_us = 0; // LIN接收超时标志
uint64_t state_start_time_us = 0;


// --- 辅助函数：校验和（支持 enhanced 或 classic） ---
static uint8_t lin_checksum(uint8_t *data, int len, uint8_t pid, bool enhanced) {
    uint16_t sum = 0;
    if (enhanced) sum += pid;
    for (int i = 0; i < len; i++) sum += data[i];
    // 折叠高位
    while (sum > 0xFF) sum = (sum & 0xFF) + (sum >> 8);
    return (uint8_t)(~sum);
}

// --- 打印帧内容（调试用） ---
static void print_lin_frame(uint8_t pid, uint8_t *data, int len) {
    packet_t linPacket;
    linPacket.id = pid & 0x3F;
    linPacket.dlc = len;
    linPacket.rtr = 0;
    linPacket.ide = 0;
    if(len>0)
    memcpy(linPacket.dataArray, data, len);
    printPacket(2,&linPacket);
    //printf("[LIN] ID=0x%02X LEN=%d DATA=", pid & 0x3F, len);
    //for (int i = 0; i < len; i++) printf(" %02X", data[i]);
    //printf("\n");
}

// 在主循环里调用，负责超时检测
void lin_poll_timeout() {
    if(!linCheckBitrateSuccess)
        return;
    uint64_t now = time_us_64();
    if ((state_start_time_us!=0) && (lin_state == LIN_RECV_DATA||lin_state == LIN_RECV_CHECKSUM)) {
        //printf("[LIN]  state_start_time_us = %" PRIu64 "  ,x = %" PRIu64 " ,lin_flash_timeout_us = %ld ,frame_pos = %d\n",state_start_time_us,(now - state_start_time_us),lin_flash_timeout_us,frame_pos);
        if ((now - state_start_time_us) > lin_flash_timeout_us) {
            //printf("[LIN]  Time OUT !!!! state_start_time_us = %" PRIu64 "  ,x = %" PRIu64 " ,lin_flash_timeout_us = %ld ,frame_pos = %d\n",state_start_time_us,(now - state_start_time_us),lin_flash_timeout_us,frame_pos);
            //printf("[LIN] timeout in state %ld  now %lu  state_start_time_us %lu \n",lin_flash_timeout_us,now,state_start_time_us);
            //lin_enter_state(LIN_WAIT_BREAK);
            //frame_pos = 0;
            if(frame_pos > 1){
                uint8_t calc_e = lin_checksum(frame_buf, frame_pos-1, current_pid, true);
                uint8_t calc_c = lin_checksum(frame_buf, frame_pos-1, current_pid, false);
                //frame_buf[frame_pos-1] = 
                //printf("[LIN]  frame_buf[frame_pos-1] = %02X  calc_e %02X  calc_c %02X \n",frame_buf[frame_pos-1],calc_e,calc_c);
                if(frame_buf[frame_pos-1] == calc_e || frame_buf[frame_pos-1] == calc_c) {
                    print_lin_frame(current_pid, frame_buf, frame_pos-1);
                }
                
            }else{
                print_lin_frame(current_pid, frame_buf, 0);
            }
            
            break_seen = false;
            sync_skip_cnt = 0;
            lin_state = LIN_WAIT_BREAK;
            frame_pos = 0;
        }
    }
}

/**
 * @brief 计算UART发送一个字节所需的时间（微秒）
 * @param baud_rate 波特率（如19200）
 * @param data_bits 数据位数（通常为8）
 * @param stop_bits 停止位数（通常为1或2）
 * @param parity 奇偶校验位（0表示无，1表示有）
 * @return 发送一个字节所需的时间（微秒）
 */
uint32_t calculate_byte_transmit_time(uint32_t baud_rate, uint8_t data_bits, uint8_t stop_bits, bool parity) {
    // 计算每帧的总位数：起始位(1) + 数据位 + 奇偶校验位(0或1) + 停止位
    uint8_t total_bits_per_frame = 1 + data_bits + (parity ? 1 : 0) + stop_bits;
    
    // 计算每位的传输时间（微秒）：1秒/波特率 * 1,000,000
    // 使用整数运算：1,000,000 / 波特率
    uint32_t bit_time_microseconds = 1000000 / baud_rate;
    
    // 计算每帧的传输时间（微秒）
    uint32_t frame_time_microseconds = bit_time_microseconds * total_bits_per_frame;
    
    return frame_time_microseconds;
}

/**
 * @brief 更精确的计算函数（使用64位整数避免舍入误差）
 * @return 发送一个字节所需的时间（微秒）
 */
uint32_t calculate_byte_transmit_time_precise(uint32_t baud_rate, uint8_t data_bits, uint8_t stop_bits, bool parity) {
    // 计算每帧的总位数
    uint8_t total_bits_per_frame = 1 + data_bits + (parity ? 1 : 0) + stop_bits;
    
    // 使用64位整数计算避免溢出和精度损失
    uint64_t time_ns = (uint64_t)total_bits_per_frame * 1000000000ULL / baud_rate;
    
    // 转换为微秒（四舍五入）
    return (time_ns + 500) / 1000;
}

// --- 辅助函数：PID 奇偶校验验证 ---
static bool lin_pid_valid(uint8_t pid) {
    uint8_t id = pid & 0x3F;
    uint8_t p0 = ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01;
    uint8_t p1 = (~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5))) & 0x01;
    uint8_t calc = (id & 0x3F) | (p0 << 6) | (p1 << 7);
    return calc == pid;
}


long lin_time_bytes_us(int bytes, int baud) {
    // 1 字节 = 1 start + 8 data + 1 stop = 10 bit
    int bits_per_byte = 10;
    long bit_time_us = 1e6 / baud; // 每 bit 时间 (us)
    return bytes * bits_per_byte * bit_time_us;
}

// --- 根据 PID 的 ID 返回数据长度 ---
// NOTE: LIN 的数据长度由 ID 映射表决定（通常由应用/规范定义）。
// 这里提供一个默认实现：如需精确长度，请修改此函数以匹配你的系统。
static int lin_pid_payload_len(uint8_t id6) {
    // 例子：简单映射（默认 8 字节）。把这里替换为实际项目的映射表。
    (void)id6;
    return 8;
}



void on_uart_rx() {

    uart_hw_t *hw = uart_get_hw(LIN_UART_ID);

    // 循环处理 FIFO 中所有字节
    while (uart_is_readable(LIN_UART_ID)) {
        uint32_t rsr = hw->rsr;            // 错误状态（读到的是当次字节的错误）
        uint8_t data = hw->dr & 0xFF;      // 取出字节
        //printf("-->: 0x%02X\n", data);
        // 如果有错误标志（FE/BE/OE 等），先处理并清除
        if (rsr & (UART_UARTRSR_FE_BITS | UART_UARTRSR_BE_BITS | UART_UARTRSR_OE_BITS)) {
            if (rsr & UART_UARTRSR_FE_BITS) {
                // framing error -> 视作 Break 的指示（不一定挂在0x00）
                //printf("[LIN] Break Error (BE) detected\n");
                if(prev_data == 0x00){
                    // 连续两个 0x00，视作 Break 残影，忽略
                    //printf("[LIN] Ignore Break residual (0x00)\n");
                break_seen = true;
                sync_skip_cnt = 0;
                }
                // 注意：不要立即 return，我们仍要处理当前读出的 data（可能是 0x55）
            }
            if (rsr & UART_UARTRSR_BE_BITS) {
                //printf("[LIN] Break Error (BE) detected\n");
                //break_seen = true;
                //sync_skip_cnt = 0;
            }
            if (rsr & UART_UARTRSR_OE_BITS) {
                printf("[LIN] Overrun (OE) - FIFO overflow\n");
                // 发生 OE 时建议清空状态/重新同步
            }
            // 清错误标志（RP2040 使用写 0 清）
            hw->rsr = 0;
        }
        
        // ----- 处于 breakpoint（等待 Sync）时的逻辑 -----
        if (break_seen) {
            // 如果当前 byte 恰好是 Sync（0x55），接受它作为 Sync（无论它是否伴随 FE）
            if (data == 0x55&&!prev_data) {
                break_seen = false;
                lin_state = LIN_RECV_PID;   // 接下来读 PID
                frame_pos = 0;
                current_pid = 0;
                state_start_time_us = 0;
                // printf("[LIN] Sync detected (0x55)\n"); // 可启用调试输出
                continue; // 下个循环会读 PID（如果 FIFO 中有）
            }

            // 忽略 0x00（Break 残影），在限定次数内忽略
            if (data == 0x00 && sync_skip_cnt < MAX_SYNC_SKIP) {
                sync_skip_cnt++;
                continue;
            }
            
            //printf("[LIN] state_start_time_us 1: %lu\n", state_start_time_us);
            // 否则同步失败（不是 0x55），回到等待 Break 状态
            break_seen = false;
            lin_state = LIN_WAIT_BREAK;
            // 继续循环处理这个 data —— 它可能是随后的噪声或新的帧的一部分
            // 不直接 discard，以便不会丢失真正的数据。
        }
        prev_data = data;
        // ----- 正常状态机（在没有 break_seen 的情况下） -----
        if (lin_state == LIN_WAIT_BREAK) {
            // 没有 Break 的上下文，忽略普通字节（或根据需要打印）
            // printf("[LIN] Ignore byte 0x%02X in WAIT_BREAK\n", data);
            continue;
        }

        if (lin_state == LIN_RECV_PID) {
            current_pid = data;
            //printf("[LIN] current_pid: 0x%02X\n", current_pid);
            if (!lin_pid_valid(current_pid)) {
                printf("[LIN] PID parity error: 0x%02X\n", current_pid);
                lin_state = LIN_WAIT_BREAK;
                continue;
            }
            lin_message_count++;
            // 获取该 ID 的长度（需由应用定义）
            int id6 = current_pid & 0x3F;
            frame_len = 8;//lin_pid_payload_len(id6); // 这里简化为固定 8 字节
            frame_pos = 0;
            state_start_time_us = time_us_64();
            lin_state = LIN_RECV_DATA;
            //printf("[LIN] state_start_time_us 2: %lu\n", state_start_time_us);
            //else lin_state = LIN_RECV_CHECKSUM; // 无数据场景
            continue;
        }

        if (lin_state == LIN_RECV_DATA) {
            //printf("[LIN] get data: 0x%02X\n", data);
            if (frame_pos < (int)sizeof(frame_buf)) {
                frame_buf[frame_pos++] = data;
            } else {
                // 溢出保护
                printf("[LIN] frame buffer overflow\n");
                lin_state = LIN_WAIT_BREAK;
                continue;
            }
            if (frame_pos >= frame_len) {
                //printf("[LIN] frame_pos %d data=0x%02X\n",frame_pos,data);
                lin_state = LIN_RECV_CHECKSUM;
            }
            continue;
        }

        if (lin_state == LIN_RECV_CHECKSUM) {
            uint8_t recv_checksum = data;
            // 先尝试 enhanced（PID + data），若失败再尝试 classic（data only）
            uint8_t calc_e = lin_checksum(frame_buf, frame_len, current_pid, true);
            uint8_t calc_c = lin_checksum(frame_buf, frame_len, current_pid, false);
            //printf("[LIN] lin_checksum 0x%02X  0x%02X 0x%02X\n",calc_e,calc_c,recv_checksum);
            if (recv_checksum == calc_e || recv_checksum == calc_c) {
                // 成功
                if(recv_checksum != 0)
                    print_lin_frame(current_pid, frame_buf, frame_len);
                else
                    print_lin_frame(current_pid, frame_buf, frame_len-1);
            } else {
                printf("[LIN] checksum error ID=0x%02X calc_e=0x%02X calc_c=0x%02X recv=0x%02X\n",
                       current_pid & 0x3F, calc_e, calc_c, recv_checksum);
            }
            // 完成一次帧解析后回到等待 Break
            lin_state = LIN_WAIT_BREAK;
            frame_pos = 0;
            continue;
        }

        // 其它情况默认忽略
    } // while readable
}

// And set up and enable the interrupt handlers
void lin_init(int baudrate) {
    uart_init(LIN_UART_ID, baudrate);
    printf("!LIN_CHECK_BITRATE:%u\n", baudrate);
    gpio_set_function(LIN_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LIN_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(LIN_UART_ID, false);
    lin_flash_timeout_us = calculate_byte_transmit_time_precise(baudrate,100, 1,false); // 8 字节时间
    //printf("[LIN] lin_flash_timeout_us %ld \n",lin_flash_timeout_us);
    // 设置UART中断回调函数
    irq_set_exclusive_handler(LIN_UART_IRQ, on_uart_rx);
    irq_set_enabled(LIN_UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(LIN_UART_ID, true, false);
    lin_message_count = 0;
}
// 校验 PID
uint8_t lin_calc_pid(uint8_t id) {
    uint8_t p0 = ((id>>0) ^ (id>>1) ^ (id>>2) ^ (id>>4)) & 0x01;
    uint8_t p1 = ~((id>>1) ^ (id>>3) ^ (id>>4) ^ (id>>5)) & 0x01;
    return (id & 0x3F) | (p0 << 6) | (p1 << 7);
}
#if 0
// 校验和
uint8_t lin_checksum(uint8_t *data, int len, uint8_t pid, bool enhanced) {
    uint16_t sum = 0;
    if (enhanced) sum += pid;
    for (int i=0; i<len; i++) sum += data[i];
    while (sum > 0xFF) sum = (sum & 0xFF) + (sum >> 8);
    return (uint8_t)(~sum);
}
#endif
uint8_t is_lin_baudrate_ok(void){
    //if(lin_message_count > 5){
     return lin_message_count > 5;
}