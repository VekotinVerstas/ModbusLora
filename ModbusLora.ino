#include <ModbusMaster.h>

#define RXD2 13
#define TXD2 15
#define RXenable 14

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(RXenable, HIGH);
}

void postTransmission()
{
  digitalWrite(RXenable, LOW);
}

void modbusSetup() {
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
  pinMode(RXenable, OUTPUT);
  digitalWrite(RXenable, LOW);

  // Modbus slave ID 1
  node.begin(1, Serial2);

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void setup() {
  Serial.begin(115200);
  modbusSetup();
}

void loop() {
  float volt, amp, watt;
  uint32_t runTime;

  volt = modbusReadFloat(0x0200);
  Serial.print("V: ");
  Serial.println( volt );

  amp = modbusReadFloat(0x0202);
  Serial.print("A: ");
  Serial.println( volt );

  watt = modbusReadFloat(0x0204);
  Serial.print("W: ");
  Serial.println( volt );

  runTime = modbusReadRunTime();
  Serial.print("Running time: ");
  Serial.println( runTime );

  delay(1000);
}

float modbusReadFloat(uint16_t addr) {
  uint8_t result;
  uint16_t data[2];
  float volt;

  result = node.readHoldingRegisters(addr, 2);
  if (result != node.ku8MBSuccess) {
    Serial.print("Modbus read failed: ");
    Serial.println(String(result));
    return (-1);
  }

  data[0] = node.getResponseBuffer(1);
  data[1] = node.getResponseBuffer(0);
  memcpy(&volt, data, 4);
  return (volt);
}

/*
float modbusReadVolt() {
  return modbusReadFloat();
}
*/

uint32_t modbusReadRunTime() {
  uint8_t result;
  result = node.readHoldingRegisters(0x0280, 2);
  if (result != node.ku8MBSuccess) {
    Serial.print("Modbus read failed: ");
    Serial.println(String(result));
    return (0);
  }
  return ((uint32_t)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);

  while (Serial2.available()) {
    Serial.print("Garbage from serial buffer: ");
    Serial.println(String(Serial2.read()));
  }
}
