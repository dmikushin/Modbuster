/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).

@defgroup setup ModbusServer Object Instantiation/Initialization
@defgroup buffer ModbusServer Buffer Management
@defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
@defgroup register Modbus Function Codes for Holding/Input Registers
@defgroup constant Modbus Function Codes, Exception Codes
*/
/*

  ModbusServer.h - Arduino library for communicating with Modbus slaves
  over RS232/485 (via RTU protocol).

  Library:: Modbuster

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

  
#ifndef MODBUSTER_CLIENT_H
#define MODBUSTER_CLIENT_H

/* _____PROJECT INCLUDES_____________________________________________________ */
// include Modbus constants
#include "Modbuster.h"

namespace ModBuster {

/* _____CLASS DEFINITIONS____________________________________________________ */
/**
Arduino class library for communicating with Modbus slaves over 
RS232/485 (via RTU protocol).
*/
class ModbusClient : public ModbusBase
{
public :

    ModbusClient();
   
    void begin(uint8_t, Stream &serial);

    // slave function that conducts Modbus transactions
    bool ModbusClientTransaction(uint16_t *regs, uint8_t u8size, uint8_t& result);
    
private :

    Stream* _serial;                                             ///< reference to serial port object
    uint8_t  _u8MBSlave;                                         ///< Modbus slave (1..247) initialized in begin()
    uint8_t u8ModbusADU[ku8MaxBufferSize];                       ///< send/receive data buffer
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

#endif // MODBUSTER_ClIENT_H

/**
@example examples/Basic/Basic.pde
@example examples/PhoenixContact_nanoLC/PhoenixContact_nanoLC.pde
@example examples/RS485_HalfDuplex/RS485_HalfDuplex.ino
*/
