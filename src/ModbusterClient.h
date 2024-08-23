#ifndef MODBUSTER_CLIENT_H
#define MODBUSTER_CLIENT_H

#include "Modbuster.h"

class Stream;

namespace ModBuster {

class ModbusClient : public ModbusBase {
public:
  ModbusClient();

  void begin(uint8_t, Stream &serial);

  // slave function that conducts Modbus transactions
  bool ModbusClientTransaction(uint16_t *regs, uint8_t u8size, uint8_t &result);

private:
  Stream *_serial;    ///< reference to serial port object
  uint8_t _u8MBSlave; ///< Modbus slave (1..247) initialized in begin()
  uint8_t u8ModbusADU[ku8MaxBufferSize]; ///< send/receive data buffer
  uint8_t u8ModbusADUSize = 0;

  uint8_t _u8TransmitBufferIndex;
  uint16_t u16TransmitBufferLength;
  uint8_t _u8ResponseBufferIndex;
  uint8_t _u8ResponseBufferLength;

  void process_FC1(uint16_t *regs, uint8_t u8size);
  void process_FC3(uint16_t *regs, uint8_t u8size);
  void process_FC5(uint16_t *regs, uint8_t u8size);
  void process_FC6(uint16_t *regs, uint8_t u8size);
  void process_FC15(uint16_t *regs, uint8_t u8size);
  void process_FC16(uint16_t *regs, uint8_t u8size);

  void sendTxBuffer();
};

} // namespace ModBuster

#endif // MODBUSTER_CLIENT_H

