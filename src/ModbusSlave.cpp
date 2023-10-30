/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).
*/
/*

  ModbusSlave.cpp - Arduino library for communicating with Modbus slaves
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


/* _____PROJECT INCLUDES_____________________________________________________ */
#include "ModbusSlave.h"

/* _____PROJECT INCLUDES_____________________________________________________ */
// functions to calculate Modbus Application Data Unit CRC
#include "util/crc16_.h"

// functions to manipulate words
#include "util/word.h"

using namespace ModBuster;

/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object; initialize it using ModbusSlave::begin().

@ingroup setup
*/
ModbusSlave::ModbusSlave(void)
{
  _idle = 0;
  _preTransmission = 0;
  _postTransmission = 0;
}

#ifndef debugSerialPort
#define debugSerialPort Serial
#endif

/**
Initialize class object.

Assigns the Modbus slave ID and serial port.
Call once class has been instantiated, typically within setup().

@param slave Modbus slave ID (1..255)
@param &serial reference to serial port object (Serial, Serial1, ... Serial3)
@ingroup setup
*/
void ModbusSlave::begin(uint8_t slave, Stream &serial)
{
//  txBuffer = (uint16_t*) calloc(ku8MaxBufferSize, sizeof(uint16_t));
  _u8MBSlave = slave;
  _serial = &serial;
  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;
  
#if __MODBUSMASTER_DEBUG__
  pinMode(__MODBUSMASTER_DEBUG_PIN_A__, OUTPUT);
  pinMode(__MODBUSMASTER_DEBUG_PIN_B__, OUTPUT);
#endif
}


void ModbusSlave::beginTransmission(uint16_t u16Address)
{
  _u16WriteAddress = u16Address;
  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;
}










uint8_t ModbusSlave::available(void)
{
  return _u8ResponseBufferLength - _u8ResponseBufferIndex;
}


uint16_t ModbusSlave::receive(void)
{
  if (_u8ResponseBufferIndex < _u8ResponseBufferLength)
  {
    return _u16ResponseBuffer[_u8ResponseBufferIndex++];
  }
  else
  {
    return 0xFFFF;
  }
}








/**
Set idle time callback function (cooperative multitasking).

This function gets called in the idle time between transmission of data
and response from slave. Do not call functions that read from the serial
buffer that is used by ModbusMaster. Use of i2c/TWI, 1-Wire, other
serial ports, etc. is permitted within callback function.

@see ModbusSlave::ModbusMasterTransaction()
*/
void ModbusSlave::idle(void (*idle)())
{
  _idle = idle;
}

/**
Set pre-transmission callback function.

This function gets called just before a Modbus message is sent over serial.
Typical usage of this callback is to enable an RS485 transceiver's
Driver Enable pin, and optionally disable its Receiver Enable pin.

@see ModbusSlave::ModbusMasterTransaction()
@see ModbusSlave::postTransmission()
*/
void ModbusSlave::preTransmission(void (*preTransmission)())
{
  _preTransmission = preTransmission;
}

/**
Set post-transmission callback function.

This function gets called after a Modbus message has finished sending
(i.e. after all data has been physically transmitted onto the serial
bus).

Typical usage of this callback is to enable an RS485 transceiver's
Receiver Enable pin, and disable its Driver Enable pin.

@see ModbusSlave::ModbusMasterTransaction()
@see ModbusSlave::preTransmission()
*/
void ModbusSlave::postTransmission(void (*postTransmission)())
{
  _postTransmission = postTransmission;
}


/**
Retrieve data from response buffer.

@see ModbusSlave::clearResponseBuffer()
@param u8Index index of response buffer array (0x00..0x3F)
@return value in position u8Index of response buffer (0x0000..0xFFFF)
@ingroup buffer
*/
uint16_t ModbusSlave::getResponseBuffer(uint8_t u8Index)
{
  if (u8Index < ku8MaxBufferSize)
  {
    return _u16ResponseBuffer[u8Index];
  }
  else
  {
    return 0xFFFF;
  }
}


/**
Clear Modbus response buffer.

@see ModbusSlave::getResponseBuffer(uint8_t u8Index)
@ingroup buffer
*/
void ModbusSlave::clearResponseBuffer()
{
  uint8_t i;
  
  for (i = 0; i < ku8MaxBufferSize; i++)
  {
    _u16ResponseBuffer[i] = 0;
  }
}


/**
Place data in transmit buffer.

@see ModbusSlave::clearTransmitBuffer()
@param u8Index index of transmit buffer array (0x00..0x3F)
@param u16Value value to place in position u8Index of transmit buffer (0x0000..0xFFFF)
@return 0 on success; exception number on failure
@ingroup buffer
*/
uint8_t ModbusSlave::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
{
  if (u8Index < ku8MaxBufferSize)
  {
    _u16TransmitBuffer[u8Index] = u16Value;
    return ku8MBSuccess;
  }
  else
  {
    return ku8MBIllegalDataAddress;
  }
}


/**
Clear Modbus transmit buffer.

@see ModbusSlave::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
@ingroup buffer
*/
void ModbusSlave::clearTransmitBuffer()
{
  uint8_t i;
  
  for (i = 0; i < ku8MaxBufferSize; i++)
  {
    _u16TransmitBuffer[i] = 0;
  }
}

/* _____PRIVATE FUNCTIONS____________________________________________________ */
/**
Modbus slave transaction engine.
This method checks if there is any incoming query
Afterwards, it would shoot a validation routine plus a register query
Avoid any delay() function !!!!
After a successful frame between the Master and the Slave, the time-out timer is reset.
Sequence:
  - poll for master request
  - evaluate/disassemble request
  - return status (success/exception)

@param *regs register table for communication exchange
@param u8size size of the register table
@return 0 on success; exception number on failure
*/
uint8_t ModbusSlave::ModbusSlaveTransaction(uint16_t *regs, uint8_t u8size)
{
  uint8_t u8ModbusADU[256];
  uint8_t u8ModbusADUSize = 0;
  uint8_t u8MBStatus = ku8MBSuccess;

  if (!_serial->available())
    return u8MBStatus;
  
  // loop until we run out of time or bytes, or an error occurs
  uint8_t u8BytesLeft = 8;
  uint32_t u32StartTime = millis();
  while (u8BytesLeft && !u8MBStatus)
  {
    if (_serial->available())
    { uint8_t ch;
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, true);
#endif
      ch = _serial->read();

#ifdef MODBUS_DEBUG       
          if (ch < 15) debugSerialPort.print("0");
          debugSerialPort.print (ch,HEX);
          debugSerialPort.print("<");
#endif         
      
      if ((ch == _u8MBSlave) || u8ModbusADUSize)
        {
        u8ModbusADU[u8ModbusADUSize++]=ch;
        u8BytesLeft--;
        }
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, false);
#endif
    }
    else
    {
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_B__, true);
#endif
      if (_idle)
      {
        _idle();
      }
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_B__, false);
#endif
    }
    
    // evaluate slave ID, function code once enough bytes have been read
    if (u8ModbusADUSize == 5)
    {
      uint8_t u8MBFunction = u8ModbusADU[1] & 0x7F;
      
      // check whether Modbus exception occurred; return Modbus Exception Code
      if (bitRead(u8ModbusADU[1], 7))
      {
        u8MBStatus = u8ModbusADU[2];
        break;
      }
      
      // evaluate returned Modbus function code
      switch(u8MBFunction)
      {
        case ku8MBReadCoils:
        case ku8MBReadDiscreteInputs:
        case ku8MBReadInputRegisters:
        case ku8MBReadHoldingRegisters:
        case ku8MBReadWriteMultipleRegisters:
          u8BytesLeft = u8ModbusADU[2];
          break;
          
        case ku8MBWriteSingleCoil:
        case ku8MBWriteMultipleCoils:
        case ku8MBWriteSingleRegister:
        case ku8MBWriteMultipleRegisters:
          u8BytesLeft = 3;
          break;
          
        case ku8MBMaskWriteRegister:
          u8BytesLeft = 5;
          break;
      }
    }
    if ((millis() - u32StartTime) > ku16MBResponseTimeout)
    {
      u8MBStatus = ku8MBResponseTimedOut;
    }
  }
  
  // verify response is large enough to inspect further
  if (!u8MBStatus && u8ModbusADUSize >= 5)
  {
    // calculate CRC
    uint16_t u16CRC = 0xFFFF;
    for (uint8_t i = 0; i < (u8ModbusADUSize - 2); i++)
    {
      u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
    }
    
    // verify CRC
    if (!u8MBStatus && (lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
      highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1]))
    {
      u8MBStatus = ku8MBInvalidCRC;
    }
  }

  // disassemble ADU into words
  if (!u8MBStatus)
  {
    // evaluate returned Modbus function code
    switch(u8ModbusADU[1])
    {
      case ku8MBReadCoils:
      case ku8MBReadDiscreteInputs:
        // load bytes into word; response bytes are ordered L, H, L, H, ...
        uint8_t i = 0;
        for ( ; i < (u8ModbusADU[2] >> 1); i++)
        {
          if (i < ku8MaxBufferSize)
          {
            _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 4], u8ModbusADU[2 * i + 3]);
          }
          
          _u8ResponseBufferLength = i;
        }
        
        // in the event of an odd number of bytes, load last byte into zero-padded word
        if (u8ModbusADU[2] % 2)
        {
          if (i < ku8MaxBufferSize)
          {
            _u16ResponseBuffer[i] = word(0, u8ModbusADU[2 * i + 3]);
          }
          
          _u8ResponseBufferLength = i + 1;
        }
        break;
        
      case ku8MBReadInputRegisters:
      case ku8MBReadHoldingRegisters:
      case ku8MBReadWriteMultipleRegisters:
        // load bytes into word; response bytes are ordered H, L, H, L, ...
        for (uint8_t i = 0; i < (u8ModbusADU[2] >> 1); i++)
        {
          if (i < ku8MaxBufferSize)
          {
            _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
          }
          
          _u8ResponseBufferLength = i;
        }
        break;
    }
  }
  
  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;
  _u8ResponseBufferIndex = 0;
  
  memcpy(regs, u8ModbusADU, sizeof(regs[0]) * u8size);

  // TODO So far, we have only received the request, now we need to respond to the query.
  
  return u8MBStatus;
}

