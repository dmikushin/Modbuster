#include "Modbuster.h"

using namespace ModBuster;

uint16_t ModBuster::crc(uint8_t* au8Buffer, uint8_t u8length)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++)
    {
        temp = temp ^ au8Buffer[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}

ModbusBase::ModbusBase() { }

void ModbusBase::preRead(void (*preRead)())
{
  _preRead = preRead;
}

void ModbusBase::idleRead(void (*idleRead)())
{
  _idleRead = idleRead;
}

void ModbusBase::postRead(void (*postRead)())
{
  _postRead = postRead;
}

void ModbusBase::preWrite(void (*preWrite)())
{
  _preWrite = preWrite;
}

void ModbusBase::postWrite(void (*postWrite)())
{
  _postWrite = postWrite;
}
