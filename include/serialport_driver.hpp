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

/****************************************************************************
* ==>> 接口
****************************************************************************/
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
    ***********************************************************************/
    uint16_t getCrc16(uint8_t *data, int len);
};

/****************************************************************************
 * ==>> 实现
 ***************************************************************************/
// Linux串口配置的结构体
struct termios serialSetting;
struct termios settingBackup;

SerialPort::SerialPort(char* in_device):device(in_device){
  nFd = portInit();
};

int SerialPort::portInit(){
  /* *** 打开串口 *** */
  // O_RDWR: 读写方式打开; O_NOCTTY: 不允许进程管理串口; O_NDELAY: 非阻塞
  nFd = open(device, O_RDWR|O_NOCTTY|O_NDELAY);
  if(-1 == nFd) {
    perror("Open Serial Port Error!\n");
    return -1;
  }
  // 恢复串口为阻塞状态
  if ((fcntl(nFd, F_SETFL, 0)) < 0) {
    perror("Fcntl F_SETFL Error!\n");
    return -1;
  }
  // 测试是否为终端设备
  if(isatty(STDIN_FILENO)==0) {
    printf("standard input is not a terminal device\n");
    return -1;
  }
  // 保存原先串口的配置
  if (tcgetattr(nFd, &settingBackup) != 0) {
    perror("tcgetattr error!\n");
    return -1;
  }

  /* *** 串口参数设置 *** */
  // 获取串口原来的参数设置
  tcgetattr(nFd, &serialSetting);
  // 设置终端为原始模式，该模式下全部的输入数据以字节为单位被处理
  cfmakeraw(&serialSetting);
  // 设置波特率，用户不能直接通过位掩码来操作
  cfsetispeed(&serialSetting, B115200);
  cfsetospeed(&serialSetting, B115200);

  // 本地连接 | 接收使能
  serialSetting.c_cflag |= (CLOCAL|CREAD);
  // 用数据位掩码清空数据位设置
  serialSetting.c_cflag &= ~CSIZE;
  // 数据位为8位
  serialSetting.c_cflag |= CS8;
  // 无校验位
  serialSetting.c_cflag &= ~PARENB;
  // 无奇偶校验位
  serialSetting.c_iflag &= ~INPCK; 
  // 清除CSTOPB，设置停止位为1 bits
  serialSetting.c_cflag &= ~CSTOPB;

  // 设置read()函数的调用方式
  // 指定读取每个字符之间的超时时间
  serialSetting.c_cc[VTIME]=0;
  // 指定所要读取字符的最小数量
  serialSetting.c_cc[VMIN]=1;

  // 清空终端未完毕的输入/输出请求及数据
  tcflush(nFd,TCIFLUSH);
  // 立即激活新配置
  if (tcsetattr(nFd, TCSANOW, &serialSetting) != 0) {
    perror("tcsetattr Error!\n");
    return -1;
  }
  return nFd;
};


/* *** 串口通讯 *** */
void SerialPort::portPush(uint8_t *tx, int txLen, uint8_t *rx, int rxLen){
/* *** 向串口发送 *** */
  write(nFd, tx, txLen);

  /* *** 从串口接收 *** */
  // 返回读到的字节数
  int nRet = read(nFd, rx, rxLen);
  if (-1 == nRet) {
    perror("Read Data Error!\n");
  }
  else {
    for (int i=0; i<rxLen; i++){
      printf("receive %02X\n", *rx);
      rx++;
    }
  }
}

/* **************************************************************************
 * 进行CRC-16/MODBUS计算
 * 生成CRC的流程为：
 * 1. 预置一个16位寄存器位FFFFH，称之为CRC寄存器。
 * 2. 把数据帧中第一个字节的8位与CRC寄存器中的低字节进行异或运算，
 *    结果存回CRC寄存器。
 * 3. 将CRC寄存器向右移1位，最高位以0填充，
 *    最低位移(least-significant byte)出并监测。
 * 4. 如果最低位为0: 重复第3步（下一次移位），如果最低位为1:
 *    则将CRC寄存器与一个预设的固定值（0A001H）进行异或运算。
 * 5. 重复第3步和第4步直到8次位移，这样就处理完了一个完整的8位。
 * 6. 重复第2步到第5步来处理下一个字节，知道处理完校验位前所有的数据。
 * 7. 最终CRC寄存器得值就是CRC的值。
 * **************************************************************************/
uint16_t getCrc16(uint8_t *arr, int len) {
  uint16_t crc = 0xFFFF, lsb;

  for (int i = 0; i < len; ++i) {
    crc ^= arr[i];
    for (int j = 0; j < 8; ++j) {
      lsb = crc & 0x0001;
      crc = crc >> 1;
      if (lsb != 0) {
        crc ^= 0xA001;
      }
    }
  }
  return crc;
}

#endif

