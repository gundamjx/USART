#include "usart.h"                     // 自定义串口操作头文件（声明结构体和函数）
#include <fcntl.h>                     // 文件控制相关宏定义（如open函数的参数）
#include <unistd.h>                    // Unix系统调用（read、write、close等）
#include <errno.h>                     // 错误码定义（用于解析系统调用的错误原因）
#include <iostream>                    // 标准输入输出（用于打印调试信息）
#include <functional>                  // 函数绑定工具（可能用于ROS回调函数绑定）
#include "geometry_msgs/msg/point.hpp" // ROS 2 点坐标消息类型（用于位置数据传输）
#include <stdint.h>  // 固定宽度整数类型（int16_t/uint8_t）
#include <termios.h> // 串口配置（termios结构体/tcgetattr/tcsetattr）


// 全局变量声明
int usart_0 = -1;                          // 串口文件描述符（-1表示未打开）
volatile sig_atomic_t usart_connected = 0; // 串口连接状态（1:已连接，0:未连接）
// bool usart_connected = false;   // 初始化连接状态为“未连接”

/////////////////////////////////////////////校验实现///////////////////////////////////////////////////////////////
// 全局校验配置（默认 CRC8：初始值0x00，多项式0x07）
ChecksumConfig g_checksum_config = {
    CHECKSUM_CRC8,  // type
    0x00,           // crc8_init
    0x07,           // crc8_poly
    0xFFFF,         // crc16_init
    0xA001          // crc16_poly (CRC-16/MODBUS)
};

/**
 * @brief 设置校验配置（约定的校验码参数）
 * @param config 校验配置结构体
 */
void set_checksum_config(const ChecksumConfig &config)
{
    g_checksum_config = config;
}

/**
 * @brief 计算奇校验校验和（对所有字节做异或后取反，使每比特位1的总数为奇数）
 * @param data 数据起始地址
 * @param len  数据长度
 * @return 校验值（8位，存于uint16_t低字节）
 */
static uint16_t calc_parity_odd(const uint8_t *data, size_t len)
{
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < len; i++)
    {
        xor_sum ^= data[i];
    }
    return (uint16_t)(uint8_t)(~xor_sum); // 取反 => 奇校验
}

/**
 * @brief 计算偶校验校验和（对所有字节做异或，每比特位1的总数为偶数）
 * @param data 数据起始地址
 * @param len  数据长度
 * @return 校验值（8位）
 */
static uint16_t calc_parity_even(const uint8_t *data, size_t len)
{
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < len; i++)
    {
        xor_sum ^= data[i];
    }
    return (uint16_t)xor_sum;
}

/**
 * @brief 计算 CRC8 校验值
 * @param data 数据起始地址
 * @param len  数据长度
 * @param init 初始值
 * @param poly 多项式
 * @return CRC8 校验值
 */
static uint8_t calc_crc8(const uint8_t *data, size_t len, uint8_t init, uint8_t poly)
{
    uint8_t crc = init;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ poly);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief 计算 CRC16 校验值（低位优先，兼容 CRC-16/MODBUS）
 * @param data 数据起始地址
 * @param len  数据长度
 * @param init 初始值
 * @param poly 多项式（低位优先形式，如 0xA001）
 * @return CRC16 校验值
 */
static uint16_t calc_crc16(const uint8_t *data, size_t len, uint16_t init, uint16_t poly)
{
    uint16_t crc = init;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (uint16_t)((crc >> 1) ^ poly);
            else
                crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/**
 * @brief 统一校验生成函数：根据配置类型计算校验值
 * @param data   待校验数据起始地址
 * @param len    数据长度
 * @param config 校验配置
 * @return 校验值（16位；奇偶/CRC8 时结果在低字节）
 */
uint16_t calc_checksum(const uint8_t *data, size_t len, const ChecksumConfig &config)
{
    switch (config.type)
    {
    case CHECKSUM_NONE:
        return 0;
    case CHECKSUM_PARITY_ODD:
        return calc_parity_odd(data, len);
    case CHECKSUM_PARITY_EVEN:
        return calc_parity_even(data, len);
    case CHECKSUM_CRC8:
        return (uint16_t)calc_crc8(data, len, config.crc8_init, config.crc8_poly);
    case CHECKSUM_CRC16:
        return calc_crc16(data, len, config.crc16_init, config.crc16_poly);
    default:
        return 0;
    }
}

/**
 * @brief 初始化串口设备
 * @param bt 波特率（如9600、9600等）
 * @return 成功返回true，失败返回false
 */
bool usart_init(int bt)
{
    // 修改串口设备权限（临时授予当前用户访问权限）
    system("sudo -S chown gundamjx /dev/ttyACM0"); // 替换为实际设备路径和用户名

    struct sigaction saio; // 信号处理结构体（用于配置SIGIO信号的处理方式）

    // 打开串口设备
    // 参数1：设备路径（通过ID定位，比/dev/ttyACM1更稳定）
    // 参数2：打开方式（O_RDWR:读写, O_NOCTTY:不作为控制终端, O_NONBLOCK:非阻塞模式）
    usart_0 = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (usart_0 <= 0)
    { // 打开失败（文件描述符<=0）
      //  std::cerr << "Can not open serial port: " << strerror(errno) << std::endl;
        return false;
    }

    // 配置SIGIO信号处理：忽略该信号（避免串口I/O事件触发默认处理）
    saio.sa_handler = SIG_IGN;     // 信号处理函数：忽略信号
    sigemptyset(&saio.sa_mask);    // 清空信号掩码
    saio.sa_flags = 0;             // 无特殊标志
    saio.sa_restorer = NULL;       // 已废弃字段，设为NULL
    sigaction(SIGIO, &saio, NULL); // 应用信号配置

    // 清除非阻塞标志（将串口设为阻塞模式，与open时的O_NONBLOCK抵消）
    if (fcntl(usart_0, F_SETFL, 0) < 0)
    {
        perror("Failed to set serial port flags"); // 打印错误信息
        close(usart_0);                            // 关闭串口
        usart_0 = -1;                              // 重置文件描述符
        return false;
    }

    // 配置串口通信参数（波特率、数据位等）
    struct termios newtio; // 串口属性结构体
    // 调用set_opt函数配置参数，失败则清理资源
    if (set_opt(newtio, usart_0, bt, 8, 'N', 1) < 0)
    {
        close(usart_0);
        usart_0 = -1;
        return false;
    }

    usart_connected = 1; // 标记串口已连接
    std::cout << "Serial port opened successfully" << std::endl;
    return true;
}

/**
 * @brief 配置串口通信参数
 * @param newtio 串口属性结构体（传出参数，保存配置结果）
 * @param fd 串口文件描述符
 * @param nSpeed 波特率
 * @param nBits 数据位（7或8）
 * @param nEvent 校验位（'O'奇校验, 'E'偶校验, 'N'无校验）
 * @param nStop 停止位（1或2）
 * @return 成功返回0，失败返回-1
 */
int set_opt(struct termios newtio, int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    newtio.c_cflag |= CLOCAL | CREAD; // CLOCAL:忽略调制解调器控制线, CREAD:启用接收
    newtio.c_cflag &= ~CSIZE;         // 清除数据位设置（准备重新配置）

    // 配置数据位
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break; // 7位数据位
    case 8:
        newtio.c_cflag |= CS8;
        break; // 8位数据位（最常用）
    }

    // 配置校验位
    switch (nEvent)
    {
    case 'O':                               // 奇校验
        newtio.c_cflag |= PARENB | PARODD;  // PARENB:启用校验, PARODD:奇校验
        newtio.c_iflag |= (INPCK | ISTRIP); // INPCK:启用校验检测, ISTRIP:剥离校验位
        break;
    case 'E': // 偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;  // 启用校验
        newtio.c_cflag &= ~PARODD; // 偶校验（清除奇校验标志）
        break;
    case 'N':                      // 无校验（最常用）
        newtio.c_cflag &= ~PARENB; // 禁用校验
        break;
    }

    // 配置波特率
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break; // 默认9600
    }

    // 配置停止位
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB; // 1位停止位（清除2位停止位标志）
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB; // 2位停止位

    // 配置超时和最小读取字节数
    newtio.c_cc[VTIME] = 0;  // 非阻塞模式（立即返回）
    newtio.c_cc[VMIN] = 0;   // 最小读取字节数：0（有数据就读，没数据立即返回）
    // 禁用规范模式（原始数据模式）
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 关闭规范模式、回显等
    newtio.c_oflag &= ~OPOST;                          // 关闭输出处理（不转换换行等字符）

    tcflush(fd, TCIFLUSH); // 刷新输入缓冲区（清除未读取的旧数据）
    // 应用配置（TCSANOW:立即生效）
    if ((tcsetattr(fd, TCSANOW, &newtio)) < 0)
    {
        perror("Failed to set serial port attributes");
        return -1;
    }
    return 0;
}

/**
 * @brief 信号处理函数（处理SIGPIPE信号）
 * @param sig 信号值（此处为SIGPIPE，表示管道破裂）
 * @note 当串口连接意外断开时可能触发此信号
 */
void handle_sigpipe(int sig)
{
    usart_connected = 0; // 标记连接断开
    if (usart_0 > 0)
    { // 关闭串口并重置文件描述符
        close(usart_0);
        usart_0 = -1;
    }
    std::cerr << "Serial connection lost" << std::endl;
}

/**
 * @brief 通过串口发送结构体数据（自动生成并填入校验位）
 * @param data 要发送的SendData结构体（包含命令和参数）
 */
void bluetooth_send(const SendData &data)
{
    // 若连接断开，尝试重新初始化串口
    if (!usart_connected)
    {
        if (!usart_init(115200))
        { // 重连失败则返回
            std::cerr << "Failed to reconnect serial port" << std::endl;
            return;
        }
    }

    // 拷贝一份数据以便填充校验位（不修改入参）
    SendData packet = data;

    // 计算校验区域：从帧头开始到校验位字段之前（不含校验位本身和帧尾）
    // checksum 位于 field 内，用指针差计算偏移（兼容嵌套成员）
    size_t checksum_offset = (size_t)((uint8_t *)&packet.sendData.checksum - (uint8_t *)&packet);
    packet.sendData.checksum = calc_checksum((const uint8_t *)&packet, checksum_offset, g_checksum_config);

    // 发送结构体数据（直接将结构体的二进制数据写入串口）
    // 参数：文件描述符、数据地址、数据大小（结构体大小）
    ssize_t bytes_written = write(usart_0, &packet, sizeof(SendData));
    if (bytes_written < 0)
    { // 发送失败（如设备断开）
        std::cerr << "Failed to write to serial port: " << strerror(errno) << std::endl;
        handle_sigpipe(0); // 手动触发连接断开处理
    }
    else if (bytes_written != sizeof(SendData))
    { // 发送不完整（部分字节未发送）
        std::cerr << "Incomplete write to serial port" << std::endl;
    }
}

// 全局接收缓冲区
uint8_t g_rx_buffer[256] = {0};
size_t g_rx_buffer_index = 0;

/**
 * @brief 通过串口接收结构体数据（带帧同步）
 * @param receive_data 用于存储接收数据的ReceiveData结构体（引用传递）
 * @return 成功返回true，失败返回false
 */
bool bluetooth_receive(ReceiveData &receive_data)
{
    if (!usart_connected)
    {
        if (!usart_init(115200))
        {
            return false;
        }
    }

    const uint8_t HEAD = 0x65;
    const uint8_t TAIL = 0x57;
    const size_t FRAME_SIZE = sizeof(ReceiveData);

    // ========== 第1步：从串口读取新数据到缓冲区 ==========
    uint8_t temp_buf[64];
    ssize_t bytes_read = read(usart_0, temp_buf, sizeof(temp_buf));

    if (bytes_read < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            // 无新数据，尝试处理缓冲区中的旧数据
            bytes_read = 0;
        }
        else
        {
            std::cerr << "串口读错误: " << strerror(errno) << std::endl;
            handle_sigpipe(0);
            return false;
        }
    }

    // ========== 第2步：新数据追加到循环缓冲区 ==========
    for (ssize_t i = 0; i < bytes_read; i++)
    {
        g_rx_buffer[g_rx_buffer_index++] = temp_buf[i];
        if (g_rx_buffer_index >= 256)
        {
            g_rx_buffer_index = 0; // 环形缓冲区回绕
        }
    }

    // ========== 第3步：逐字节搜索帧头并校验完整帧 ==========
    while (g_rx_buffer_index >= FRAME_SIZE)
    {
        // 如果缓冲区首字节不是帧头，逐字节丢弃直到找到帧头
        if (g_rx_buffer[0] != HEAD)
        {
            std::cerr << "⚠️  丢弃无效字节: 0x" << std::hex << (int)g_rx_buffer[0] << std::dec << std::endl;
            // 移除首字节，缓冲区前移
            for (size_t k = 0; k < g_rx_buffer_index - 1; k++)
            {
                g_rx_buffer[k] = g_rx_buffer[k + 1];
            }
            g_rx_buffer_index--;
            continue;
        }

        // ========== 第4步：找到帧头，检查帧尾是否正确 ==========
        if (g_rx_buffer[FRAME_SIZE - 1] != TAIL)
        {
            std::cerr << "⚠️  帧尾错误，期望0x" << std::hex << (int)TAIL 
                      << "，获得0x" << (int)g_rx_buffer[FRAME_SIZE - 1] << std::dec << std::endl;
            // 帧尾错误，丢弃该帧头，继续寻找下一个
            for (size_t k = 0; k < g_rx_buffer_index - 1; k++)
            {
                g_rx_buffer[k] = g_rx_buffer[k + 1];
            }
            g_rx_buffer_index--;
            continue;
        }

        // ========== 第5步：帧头尾都正确，提取数据 ==========
        std::memcpy(&receive_data, g_rx_buffer, FRAME_SIZE);

        // ========== 第6步：自动校验（校验位之前的数据） ==========
        size_t rx_checksum_offset = (size_t)((uint8_t *)&receive_data.receiveData.checksum - (uint8_t *)&receive_data);
        uint16_t calc_crc = calc_checksum((const uint8_t *)&receive_data, rx_checksum_offset, g_checksum_config);
        if (calc_crc != receive_data.receiveData.checksum)
        {
            std::cerr << "⚠️  校验失败：计算=0x" << std::hex << calc_crc
                      << "，接收=0x" << receive_data.receiveData.checksum << std::dec << std::endl;
            // 校验失败，丢弃该帧头，继续寻找下一帧
            for (size_t k = 0; k < g_rx_buffer_index - 1; k++)
            {
                g_rx_buffer[k] = g_rx_buffer[k + 1];
            }
            g_rx_buffer_index--;
            continue;
        }

        // 移除已处理的帧
        for (size_t k = 0; k < g_rx_buffer_index - FRAME_SIZE; k++)
        {
            g_rx_buffer[k] = g_rx_buffer[k + FRAME_SIZE];
        }
        g_rx_buffer_index -= FRAME_SIZE;

        std::cout << "✅ 接收成功：data0=" << receive_data.receiveData.data0
                  << ", data1=" << receive_data.receiveData.data1
                  << ", data2=" << receive_data.receiveData.data2 << std::endl;
        return true;
    }

    // 缓冲区中数据不足一帧，继续等待
    return false;
}
