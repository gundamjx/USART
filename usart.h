#ifndef USART_H
#define USART_H

#include <stdint.h>  // 用于 uint8_t, int16_t 等类型
#include <stddef.h>  // 用于 offsetof
#include <termios.h> // 串口控制定义
#include <errno.h>   // 错误号定义
#include <sys/ioctl.h>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <csignal>

/////////////////////////////////////////////校验配置///////////////////////////////////////////////////////////////
// 校验类型：无奇偶校验/奇校验/偶校验/CRC8/CRC16
typedef enum {
    CHECKSUM_NONE = 0,      // 无校验
    CHECKSUM_PARITY_ODD,    // 奇校验（异或和取反）
    CHECKSUM_PARITY_EVEN,   // 偶校验（异或和）
    CHECKSUM_CRC8,          // CRC8 校验
    CHECKSUM_CRC16          // CRC16 校验
} ChecksumType;

// 校验配置（约定的校验参数，可在运行时设置）
typedef struct {
    ChecksumType type;      // 校验类型
    uint8_t  crc8_init;     // CRC8 初始值
    uint8_t  crc8_poly;     // CRC8 多项式
    uint16_t crc16_init;    // CRC16 初始值
    uint16_t crc16_poly;    // CRC16 多项式
} ChecksumConfig;

// 全局校验配置（默认 CRC8，初始值0x00，多项式0x07）
extern ChecksumConfig g_checksum_config;

// 设置校验配置（约定的校验码参数）
void set_checksum_config(const ChecksumConfig &config);

// 统一校验生成函数：根据配置类型计算校验值
// data: 待校验数据起始地址，len: 数据长度
uint16_t calc_checksum(const uint8_t *data, size_t len, const ChecksumConfig &config);
/////////////////////////////////////////////串口初始化配置///////////////////////////////////////////////////////////////
extern volatile sig_atomic_t usart_connected;
// 串口初始化函数
bool usart_init(int baud_rate);

// 串口配置函数
int set_opt(struct termios newtio, int fd, int nSpeed, int nBits, char nEvent, int nStop);

/////////////////////////////////////////////收发数据包定义///////////////////////////////////////////////////////////////

// 发送数据联合体：完整一帧（帧头 + 9个float + 校验位 + 帧尾）与字节数组共享内存
typedef union {
    struct {
        uint8_t frame_start = 0x65;   // 帧头
        float data0;
        float data1;
        float data2;
        float data3;
        float data4;
        float data5;
        float data6;
        float data7;
        float data8;
        uint16_t checksum = 0;        // 校验位
        uint8_t frame_end = 0x57;     // 帧尾
    }__attribute__((packed)) sendData;  // packed 避免 frame_start 与 float 之间插入填充
    uint8_t bytes[40];                // 1 + 9*4 + 2 + 1 = 40 字节
} sendDataUnion;

// 接收数据联合体：完整一帧（帧头 + 5个float + 校验位 + 帧尾）与字节数组共享内存
typedef union {
    struct {
        uint8_t frame_start = 0x65;   // 帧头
        float data0;
        float data1;
        float data2;
        float data3;
        float data4;
        uint16_t checksum = 0;        // 校验位
        uint8_t frame_end = 0x57;     // 帧尾
    } __attribute__((packed)) receiveData;
    uint8_t bytes[24];                // 1 + 5*4 + 2 + 1 = 24 字节
} receiveDataUnion;

//packed补齐规则，每个成员的起始地址必须是他宽度的整数倍，否则在前面插入填充字节，
//且整个包的长度必须是最大成员宽的的整数倍

// 发送/接收数据包使用各自的联合体
typedef sendDataUnion SendData;
typedef receiveDataUnion ReceiveData;

//链接断开处理函数
void handle_sigpipe(int sig);
// 数据发送函数
void bluetooth_send(const SendData &data);

// 帧同步用的接收缓冲区
#define RX_BUFFER_SIZE 256
extern uint8_t g_rx_buffer[RX_BUFFER_SIZE];
extern size_t g_rx_buffer_index;

//数据接受函数
bool bluetooth_receive(ReceiveData &receive_data);


#endif