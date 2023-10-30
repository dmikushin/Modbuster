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
ModbusSlave::ModbusSlave(void) : ModbusBase() { }

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
@param 0 on success; exception number on failure
@return true, if request has been handled; false otherwiser
*/
bool ModbusSlave::ModbusSlaveTransaction(uint16_t *regs, uint8_t u8size, uint8_t& u8MBStatus)
{
  u8MBStatus = ku8MBSuccess;

  if (!_serial->available())
    return false;

#ifdef MODBUS_DEBUG       
  debugSerialPort.println();
#endif
 
  // loop until the frame is sealed by a T35 delay.
  uint8_t u8BytesLeft = 8;
  const uint8_t T35 = 5;
  uint32_t u32StartTime = millis();
  do
  {
    if (!_serial->available()) continue;

#if __MODBUSMASTER_DEBUG__
    digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, true);
#endif
    uint8_t ch = _serial->read();

#ifdef MODBUS_DEBUG       
    if (ch < 15) debugSerialPort.print("0");
    debugSerialPort.print(ch, HEX);
    debugSerialPort.print("<");
#endif         
  
    u8ModbusADU[u8ModbusADUSize++] = ch;
    u8BytesLeft--;
    u32StartTime = millis();

#if __MODBUSMASTER_DEBUG__
    digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, false);
#endif
    
  }
  while ((millis() - u32StartTime) < T35);

#ifdef MODBUS_DEBUG       
  debugSerialPort.println();
#endif

  uint8_t id = u8ModbusADU[ID];
  if (id != _u8MBSlave)
    return false;

  // calculate CRC
  uint16_t u16CRC = crc(u8ModbusADU, u8ModbusADUSize - 2);

  Serial.println(u16CRC, HEX);
  Serial.println(u8ModbusADU[u8ModbusADUSize - 2], HEX);
  Serial.println(u8ModbusADU[u8ModbusADUSize - 1], HEX);

  // verify CRC
  if (highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
    lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1])
  {
    u8MBStatus = ku8MBInvalidCRC;
    return true;
  }

  // Process request and prepare response of in the same buffer.
  uint8_t u8MBFunction = u8ModbusADU[FUNC];
  switch (u8MBFunction)
  {
  case ku8MBReadCoils:
  case ku8MBReadDiscreteInputs:
    process_FC1(regs, u8size);
    break;
  case ku8MBReadInputRegisters:
  case ku8MBReadHoldingRegisters:
  case ku8MBReadWriteMultipleRegisters:
    process_FC3(regs, u8size);
    break;
  case ku8MBWriteSingleCoil:
    process_FC5(regs, u8size);
    break;
  case ku8MBWriteSingleRegister:
    process_FC6(regs, u8size);
    break;
  case ku8MBWriteMultipleCoils:
    process_FC15(regs, u8size);
    break;
  case ku8MBWriteMultipleRegisters:
    process_FC16(regs, u8size);
    break;
  default:
    break;
  }
  
  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;
  _u8ResponseBufferIndex = 0;

  // flush receive buffer before transmitting request
  while (_serial->read() != -1)
    continue;

  // transmit response
  if (_preTransmission)
  {
    _preTransmission();
  }
  
  sendTxBuffer();

  if (_postTransmission)
  {
    _postTransmission();
  }
  
  return true;
}

/**
 * @brief
 * This method processes functions 1 & 2
 * This method reads a bit array and transfers it to the master
 *
 * @return u8ModbusADUSize Response to master length
 * @ingroup discrete
 */
void ModbusSlave::process_FC1( uint16_t *regs, uint8_t /*u8size*/ )
{
    uint8_t u8currentRegister, u8currentBit, u8bytesno, u8bitsno;
    uint16_t u16currentCoil, u16coil;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word( u8ModbusADU[ ADD_HI ], u8ModbusADU[ ADD_LO ] );
    uint16_t u16Coilno = word( u8ModbusADU[ NB_HI ], u8ModbusADU[ NB_LO ] );

    // put the number of bytes in the outcoming message
    u8bytesno = (uint8_t) (u16Coilno / 8);
    if (u16Coilno % 8 != 0) u8bytesno ++;
    u8ModbusADU[ ADD_HI ]  = u8bytesno;
    u8ModbusADUSize         = ADD_LO;
    u8ModbusADU[ u8ModbusADUSize + u8bytesno - 1 ] = 0;

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;

    // Clear all data bits in outgoing message.
    memset(u8ModbusADU + u8ModbusADUSize, 0, sizeof(u8ModbusADU) - u8ModbusADUSize);

    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) (u16coil % 16);

        bitWrite(
            u8ModbusADU[ u8ModbusADUSize ],
            u8bitsno,
            bitRead( regs[ u8currentRegister ], u8currentBit ) );
        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8ModbusADUSize++;
        }
    }

    // send outcoming message
    if (u16Coilno % 8 != 0) u8ModbusADUSize++;
}

/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the master
 *
 * @return u8ModbusADUSize Response to master length
 * @ingroup register
 */
void ModbusSlave::process_FC3( uint16_t *regs, uint8_t /*u8size*/ )
{

    uint8_t u8StartAdd = word( u8ModbusADU[ ADD_HI ], u8ModbusADU[ ADD_LO ] );
    uint8_t u8regsno = word( u8ModbusADU[ NB_HI ], u8ModbusADU[ NB_LO ] );
    uint8_t i;

    u8ModbusADU[ 2 ]       = u8regsno * 2;
    u8ModbusADUSize         = 3;

    for (i = u8StartAdd; i < u8StartAdd + u8regsno; i++)
    {
        u8ModbusADU[ u8ModbusADUSize ] = highByte(regs[i]);
        u8ModbusADUSize++;
        u8ModbusADU[ u8ModbusADUSize ] = lowByte(regs[i]);
        u8ModbusADUSize++;
    }
}

/**
 * @brief
 * This method processes function 5
 * This method writes a value assigned by the master to a single bit
 *
 * @return u8ModbusADUSize Response to master length
 * @ingroup discrete
 */
void ModbusSlave::process_FC5( uint16_t *regs, uint8_t /*u8size*/ )
{
    uint8_t u8currentRegister, u8currentBit;
    uint8_t u8CopyBufferSize;
    uint16_t u16coil = word( u8ModbusADU[ ADD_HI ], u8ModbusADU[ ADD_LO ] );

    // point to the register and its bit
    u8currentRegister = (uint8_t) (u16coil / 16);
    u8currentBit = (uint8_t) (u16coil % 16);

    // write to coil
    bitWrite(
        regs[ u8currentRegister ],
        u8currentBit,
        u8ModbusADU[ NB_HI ] == 0xff );


    // send answer to master
    u8ModbusADUSize = 6;
}

/**
 * @brief
 * This method processes function 6
 * This method writes a value assigned by the master to a single word
 *
 * @return u8ModbusADUSize Response to master length
 * @ingroup register
 */
void ModbusSlave::process_FC6( uint16_t *regs, uint8_t /*u8size*/ )
{

    uint8_t u8add = word( u8ModbusADU[ ADD_HI ], u8ModbusADU[ ADD_LO ] );
    uint8_t u8CopyBufferSize;
    uint16_t u16val = word( u8ModbusADU[ NB_HI ], u8ModbusADU[ NB_LO ] );

    regs[ u8add ] = u16val;

    // keep the same header
    u8ModbusADUSize = ku8ResponseSize;
}

/**
 * @brief
 * This method processes function 15
 * This method writes a bit array assigned by the master
 *
 * @return u8ModbusADUSize Response to master length
 * @ingroup discrete
 */
void ModbusSlave::process_FC15( uint16_t *regs, uint8_t /*u8size*/ )
{
    uint8_t u8currentRegister, u8currentBit, u8frameByte, u8bitsno;
    uint16_t u16currentCoil, u16coil;
    boolean bTemp;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word( u8ModbusADU[ ADD_HI ], u8ModbusADU[ ADD_LO ] );
    uint16_t u16Coilno = word( u8ModbusADU[ NB_HI ], u8ModbusADU[ NB_LO ] );

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;
    u8frameByte = 7;
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {

        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) (u16coil % 16);

        bTemp = bitRead(
                    u8ModbusADU[ u8frameByte ],
                    u8bitsno );

        bitWrite(
            regs[ u8currentRegister ],
            u8currentBit,
            bTemp );

        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8frameByte++;
        }
    }

    // send outcoming message
    // it's just a copy of the incomping frame until 6th byte
    u8ModbusADUSize = 6;
}

/**
 * @brief
 * This method processes function 16
 * This method writes a word array assigned by the master
 *
 * @return u8ModbusADUSize Response to master length
 * @ingroup register
 */
void ModbusSlave::process_FC16( uint16_t *regs, uint8_t /*u8size*/ )
{
    uint8_t u8StartAdd = u8ModbusADU[ ADD_HI ] << 8 | u8ModbusADU[ ADD_LO ];
    uint8_t u8regsno = u8ModbusADU[ NB_HI ] << 8 | u8ModbusADU[ NB_LO ];
    uint8_t i;
    uint16_t temp;

    // build header
    u8ModbusADU[ NB_HI ]   = 0;
    u8ModbusADU[ NB_LO ]   = u8regsno;
    u8ModbusADUSize         = ku8ResponseSize;

    // write registers
    for (i = 0; i < u8regsno; i++)
    {
        temp = word(
                   u8ModbusADU[ (BYTE_CNT + 1) + i * 2 ],
                   u8ModbusADU[ (BYTE_CNT + 2) + i * 2 ]);

        regs[ u8StartAdd + i ] = temp;
    }
}

/**
 * @brief
 * This method transmits u8ModbusADU to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with UCSRxA register.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @param nothing
 * @return nothing
 * @ingroup buffer
 */
void ModbusSlave::sendTxBuffer()
{
    // append CRC to message
    uint16_t u16crc = crc(u8ModbusADU, u8ModbusADUSize);
    u8ModbusADU[ u8ModbusADUSize ] = u16crc >> 8;
    u8ModbusADUSize++;
    u8ModbusADU[ u8ModbusADUSize ] = u16crc & 0x00ff;
    u8ModbusADUSize++;

#ifdef MODBUS_DEBUG       
	debugSerialPort.println();
#endif

    // transfer buffer to serial line
	for (uint8_t i = 0; i < u8ModbusADUSize; i++)
	{
		_serial->write(u8ModbusADU[i]);
    
#ifdef MODBUS_DEBUG       
		if (u8ModbusADU[i]<15) debugSerialPort.print("0");    
		debugSerialPort.print (u8ModbusADU[i],HEX);
		debugSerialPort.print(">");
#endif
	}
  
#ifdef MODBUS_DEBUG       
	debugSerialPort.println();
#endif
  
	u8ModbusADUSize = 0;
	
	// flush transmit buffer
	_serial->flush();

	while (_serial->read() >= 0)
		continue;
}

