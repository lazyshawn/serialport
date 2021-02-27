
#include "../include/serialport_driver.hpp"
#include "../include/common.h"

#define DEVICE "/dev/ttyUSB0"

int main(int argc, char** argv) {
  char device[] = "/dev/ttyUSB0";
  SerialPort robQ(device);

  uint8_t txData[] = {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0xE1};
  uint8_t rx[11];

  robQ.portPush(txData, sizeof(txData), rx, 8);
}

