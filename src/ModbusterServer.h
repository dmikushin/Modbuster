#ifndef MODBUSTER_SERVER_H
#define MODBUSTER_SERVER_H

#include "Modbuster.h"

class Stream;

namespace ModBuster {

class ModbusServer : public ModbusBase {
public:
  ModbusServer();

  void begin(uint8_t, Stream &serial);

  uint16_t getResponseTimeOut() const;
  void setResponseTimeOut(uint16_t u16MBResponseTimeout);

  uint16_t getResponseBuffer(uint8_t);
  void clearResponseBuffer();
  uint8_t setTransmitBuffer(uint8_t, uint16_t);
  void clearTransmitBuffer();

  void beginTransmission(uint16_t);
  uint8_t requestFrom(uint16_t, uint16_t);
  void sendBit(bool);
  void send(uint8_t);
  void send(uint16_t);
  void send(uint32_t);
  uint8_t available(void);
  uint16_t receive(void);

  uint8_t readCoils(uint16_t, uint16_t);
  uint8_t readDiscreteInputs(uint16_t, uint16_t);
  uint8_t readHoldingRegisters(uint16_t, uint16_t);
  uint8_t readInputRegisters(uint16_t, uint8_t);
  uint8_t writeSingleCoil(uint16_t, uint8_t);
  uint8_t writeSingleRegister(uint16_t, uint16_t);
  uint8_t writeMultipleCoils(uint16_t, uint16_t);
  uint8_t writeMultipleCoils();
  uint8_t writeMultipleRegisters(uint16_t, uint16_t);
  uint8_t writeMultipleRegisters();
  uint8_t maskWriteRegister(uint16_t, uint16_t, uint16_t);
  uint8_t readWriteMultipleRegisters(uint16_t, uint16_t, uint16_t, uint16_t);
  uint8_t readWriteMultipleRegisters(uint16_t, uint16_t);
  uint8_t ModbusRawTransaction(uint8_t *u8ModbusADU, uint8_t u8ModbusADUSize,
                               uint8_t u8BytesLeft);

private:
  Stream *_serial;    ///< reference to serial port object
  uint8_t _u8MBSlave; ///< Modbus slave (1..247) initialized in begin()
  uint16_t _u16MBResponseTimeout = ku16MBResponseTimeout; ///< Modbus timeout [milliseconds]
  uint16_t _u16ReadAddress;  ///< slave register from which to read
  uint16_t _u16ReadQty;      ///< quantity of words to read
  uint16_t _u16ResponseBuffer[ku8MaxBufferSize]; ///< buffer to store Modbus
                                                 ///< slave response; read via
                                                 ///< GetResponseBuffer()
  uint16_t _u16WriteAddress; ///< slave register to which to write
  uint16_t _u16WriteQty;     ///< quantity of words to write
  uint16_t _u16TransmitBuffer[ku8MaxBufferSize]; ///< buffer containing data to
                                            ///< transmit to Modbus slave; set
                                            ///< via SetTransmitBuffer()
  uint16_t *txBuffer; // from Wire.h -- need to clean this up Rx
  uint8_t _u8TransmitBufferIndex;
  uint16_t u16TransmitBufferLength;
  uint16_t *rxBuffer; // from Wire.h -- need to clean this up Rx
  uint8_t _u8ResponseBufferIndex;
  uint8_t _u8ResponseBufferLength;

  // master function that conducts Modbus transactions
  uint8_t ModbusServerTransaction(uint8_t u8MBFunction);
};

} // namespace ModBuster

#endif // MODBUSTER_SERVER_H
