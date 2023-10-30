/*
 * This is a practical demo of ModbusRtu client (slave) that should work
 * on any Arduino board.
 * It defines a vector of registers starting with activity mask, followed by dummy values.
 * Upon every MODBUS client state update, the client checks the "activity" bitmask
 * set by user and increments only those values that are marked "active".
 */
#include <ModbusSlave.h>
//#include <PostNeoSWSerial.h>
#include <SoftwareSerial.h>

#define ID 1 /* 0 for master, 1-247 for slave */

#define BAUDRATE 9600

using namespace ModBuster;

const uint8_t RX = 9;
const uint8_t TX = 10;

//PostNeoSWSerial swSerial(RX, TX);
SoftwareSerial swSerial(RX, TX);

// Create instance
ModbusSlave slave;

enum ModBusRegisters
{
  // Activity bit mask:
  // 0 - device inactive
  // 1 << 1 - first value is collected
  // 1 << 2 - second value is collected
  // 1 << 3 - third, forth and fifth values are collected
  ModBusActive = 0,

  // First value.
  ModBusValue1,

  // Second value.
  ModBusValue2,

  // Third value (3 components).
  ModBusValue3,
  ModBusValue4,
  ModBusValue5,

  ModBusNumRegisters
};

// MODBUS registers states
uint16_t regs[ModBusNumRegisters];

void setup() {
  Serial.begin(BAUDRATE);
  memset(regs, 0, sizeof(regs));
  swSerial.begin(BAUDRATE);
  slave.begin(ID, swSerial);
}

void loop() {
  // Receive the given registers state update from master
  uint8_t result;
  bool processed = slave.ModbusSlaveTransaction(regs, ModBusNumRegisters, result);

  // Check if read was successful
  if (result == ModBuster::ku8MBSuccess)
  {
    if (processed)
    {
      Serial.println("MODBUS: request handled successfully");

      // Read the activation mask.
      auto activeMask = regs[ModBusActive];

      // Increment all active values.
      if (activeMask & (1 << ModBusValue1))
        regs[ModBusValue1]++;
      if (activeMask & (1 << ModBusValue2))
        regs[ModBusValue2]++;
      if (activeMask & (1 << ModBusValue3))
      {
        regs[ModBusValue3]++;
        regs[ModBusValue4]++;
        regs[ModBusValue5]++;
      }
    }
  }
  else
  {
    Serial.print("MODBUS: request handling failed: ");
    Serial.println(result, HEX);
  }

  delay(500);
}
