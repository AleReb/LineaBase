#include <SoftwareSerial.h>
#include <ModbusMaster.h>

const int SENSOR_POWER_PIN = 4;

#define RS485_RX 27
#define RS485_TX 15
#define RS485_EN 14

SoftwareSerial rs485Serial(RS485_RX, RS485_TX);
ModbusMaster node;

void preTx() {
  digitalWrite(RS485_EN, HIGH);
}

void postTx() {
  digitalWrite(RS485_EN, LOW);
}

void setup() {
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  Serial.begin(115200);
  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  rs485Serial.begin(4800);
  node.begin(1, rs485Serial);  // Dirección actual del sensor (1)
  node.preTransmission(preTx);
  node.postTransmission(postTx);

  delay(500);  // Por si requiere estabilización

  // --- CAMBIAR ID DEL SENSOR A 2 ---
  uint8_t result = node.writeSingleRegister(0x07D0, 2);
  if (result == node.ku8MBSuccess) {
    Serial.println("Sensor UV ID cambiado exitosamente a 2.");
  } else {
    Serial.printf("Fallo al cambiar ID. Error Modbus: %u\n", result);
  }
}

void loop() {
  // No hacer nada después de cambiar el ID
}
