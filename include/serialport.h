#ifndef SERIALPORT_DRIVER_H
#define SERIALPORT_DRIVER_H

// 串口通讯头文件
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>

// 自定义串口类
class SerialPort {
  public:
    // 打开串口获得的文件描述符
    int nFd;
    // 串口名称
    char* device;
    // 波特率
    int baudRate;

    // 构造函数
    SerialPort(char* device);
    // 串口初始化
    int portInit();
    // 串口通讯
    void portPush(uint8_t *tx, int txLen, uint8_t *rx, int rxLen);
    /* *********************************************************************
    * ==>> Function:
    * 进行CRC-16/MODBUS计算
    * ==>> Parameters:
    * *data: 输入数据数组指针; len: 数组长度.
    * ==>> 测试网站
    * http://www.ip33.com/crc.html
    ***********************************************************************/
    uint16_t getCrc16(uint8_t *data, int len);
};

#endif

