/**
 * @file template.c
 * @brief 串口收发使用示例（含可配置校验：奇偶校验 / CRC8 / CRC16）
 *
 * 编译命令（Linux，链接到 usart.cpp）：
 *   g++ -o template template.c usart.cpp -std=c++11
 *
 * 运行前请确认串口设备权限，或使用 sudo 运行。
 */

#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// 简单延时（毫秒）
static void delay_ms(int ms)
{
    usleep(ms * 1000);
}

// ============================================================
// 示例 1：配置校验方式（约定的校验码参数）
// ============================================================
void example_set_checksum(void)
{
    // ---- 方式 A：使用默认配置（CRC8，初始值 0x00，多项式 0x07）----
    // 无需任何调用，g_checksum_config 已默认设为 CRC8。

    // ---- 方式 B：切换为奇校验 ----
    ChecksumConfig cfg_odd = {
        CHECKSUM_PARITY_ODD, // type
        0, 0,                // crc8_init, crc8_poly (不用)
        0, 0                 // crc16_init, crc16_poly (不用)
    };
    set_checksum_config(cfg_odd);

    // ---- 方式 C：切换为偶校验 ----
    ChecksumConfig cfg_even = {CHECKSUM_PARITY_EVEN, 0, 0, 0, 0};
    set_checksum_config(cfg_even);

    // ---- 方式 D：切换为 CRC8（自定义初始值和多项式）----
    ChecksumConfig cfg_crc8 = {
        CHECKSUM_CRC8,
        0x00,   // crc8_init
        0x07,   // crc8_poly
        0, 0
    };
    set_checksum_config(cfg_crc8);

    // ---- 方式 E：切换为 CRC16（默认 CRC-16/MODBUS：初始值 0xFFFF，多项式 0xA001）----
    ChecksumConfig cfg_crc16 = {
        CHECKSUM_CRC16,
        0, 0,          // crc8 参数不用
        0xFFFF,        // crc16_init
        0xA001         // crc16_poly (低位优先形式)
    };
    set_checksum_config(cfg_crc16);

    // ---- 方式 F：关闭校验 ----
    ChecksumConfig cfg_none = {CHECKSUM_NONE, 0, 0, 0, 0};
    set_checksum_config(cfg_none);
}

// ============================================================
// 示例 2：发送一帧数据
// ============================================================
void example_send(void)
{
    // 1. 初始化串口（波特率 115200，设备路径在 usart_init 内部指定）
    if (!usart_init(115200))
    {
        printf("[ERROR] 串口初始化失败\n");
        return;
    }

    // 2. 选择校验方式（这里以 CRC8 为例）
    ChecksumConfig cfg = {CHECKSUM_CRC8, 0x00, 0x07, 0, 0};
    set_checksum_config(cfg);

    // 3. 构造发送数据（帧头/帧尾/校验位由结构体默认值和自动填充处理）
    SendData tx;
    tx.sendData.data0 = 1.0f;
    tx.sendData.data1 = 2.0f;
    tx.sendData.data2 = 3.0f;
    tx.sendData.data3 = 4.0f;
    tx.sendData.data4 = 5.0f;
    tx.sendData.data5 = 6.0f;
    tx.sendData.data6 = 7.0f;
    tx.sendData.data7 = 8.0f;
    tx.sendData.data8 = 9.0f;
    // 注意：tx.sendData.checksum 无需手动赋值，bluetooth_send 会自动计算并写入
    // 也可以通过字节数组直接赋值：tx.bytes[0] = ...

    // 4. 发送（内部自动生成校验位并加入到包中）
    bluetooth_send(tx);
    printf("[SEND] data0=%.2f data1=%.2f data2=%.2f ... data8=%.2f\n",
           tx.sendData.data0, tx.sendData.data1,
           tx.sendData.data2, tx.sendData.data8);
}

// ============================================================
// 示例 3：循环接收数据（自动校验）
// ============================================================
void example_receive_loop(void)
{
    // 1. 初始化串口
    if (!usart_init(115200))
    {
        printf("[ERROR] 串口初始化失败\n");
        return;
    }

    // 2. 配置与发送端一致的校验方式（必须一致，否则校验永远失败）
    ChecksumConfig cfg = {CHECKSUM_CRC8, 0x00, 0x07, 0, 0};
    set_checksum_config(cfg);

    // 3. 循环接收
    ReceiveData rx;
    while (1)
    {
        // bluetooth_receive 内部会：
        //   - 搜索帧头 0x65
        //   - 检查帧尾 0x57
        //   - 自动计算校验值并与包内 checksum 对比
        //   - 校验失败则丢弃该帧，继续寻找下一帧
        if (bluetooth_receive(rx))
        {
            printf("[RECV] data0=%.2f data1=%.2f data2=%.2f (checksum=0x%04X)\n",
                   rx.receiveData.data0, rx.receiveData.data1,
                   rx.receiveData.data2, rx.receiveData.checksum);
        }
        else
        {
            // 无完整帧或校验失败，短暂延时降低 CPU 占用
            delay_ms(10);
        }
    }
}

// ============================================================
// 示例 4：发送 + 接收回环测试
// ============================================================
void example_echo_test(void)
{
    if (!usart_init(115200))
    {
        printf("[ERROR] 串口初始化失败\n");
        return;
    }

    // 收发两端必须使用相同校验配置
    ChecksumConfig cfg = {CHECKSUM_CRC16, 0, 0, 0xFFFF, 0xA001};
    set_checksum_config(cfg);

    // 发送一帧
    SendData tx;
    tx.sendData.data0 = 10.5f;
    tx.sendData.data1 = 20.5f;
    tx.sendData.data2 = 0.0f;
    tx.sendData.data3 = 0.0f;
    tx.sendData.data4 = 100.0f;
    tx.sendData.data5 = 0.0f;
    tx.sendData.data6 = 0.0f;
    tx.sendData.data7 = 0.0f;
    tx.sendData.data8 = 0.0f;
    bluetooth_send(tx);
    printf("[SEND] data0=%.2f data1=%.2f data4=%.2f\n",
           tx.sendData.data0, tx.sendData.data1, tx.sendData.data4);

    // 等待对端回复
    ReceiveData rx;
    int retry = 100;
    while (retry-- > 0)
    {
        if (bluetooth_receive(rx))
        {
            printf("[RECV] data0=%.2f data1=%.2f data2=%.2f\n",
                   rx.receiveData.data0, rx.receiveData.data1, rx.receiveData.data2);
            break;
        }
        delay_ms(10);
    }
    if (retry <= 0)
    {
        printf("[WARN] 接收超时\n");
    }
}

// ============================================================
// 主函数（按需选择运行哪个示例）
// ============================================================
int main(void)
{
    printf("=== 串口收发示例（带校验）===\n");

    // 取消注释以运行对应示例：
    // example_send();          // 仅发送
    // example_receive_loop();  // 循环接收
    example_echo_test();        // 收发回环测试

    return 0;
}
