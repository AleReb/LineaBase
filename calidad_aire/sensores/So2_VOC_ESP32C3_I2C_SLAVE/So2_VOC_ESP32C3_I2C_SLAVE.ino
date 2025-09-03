/*
   I2C Slave Code: combine SO₂ and TVOC sensors
   - I2C address: 0x08
   - SO₂ sensor (SPEC ULP) on A3 (C1) and A4 (T1), non-blocking 20 s sampling
   - TVOC sensor (Winsen) via SoftwareSerial RX=0, TX=1, 9-byte packets with checksum
   - Serial debug: full sensor data every 20 s
   - Serial debug: TVOC packet details when master sends “OK”
   - On I2C request: send "<pVgas>,<TVOC_ug/m3>\n"
*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include "ULP.h"

#define I2C_SLAVE_ADDRESS 0x08
#define RX_PIN 0
#define TX_PIN 1

SoftwareSerial sensorSerial(RX_PIN, TX_PIN);

// SO₂ sensor pins & averaging periods
const int C1 = A3;
const int T1 = A4;
const int GAS_AVG_SEC = 5;
const int TEMP_AVG_SEC = 1;

SO2 sensor1(C1, T1, 47.54);

unsigned long lastSampleMillis = 0;
const unsigned long SAMPLE_INTERVAL = 20000;

volatile bool ackReceived = false;
uint16_t vocConcentration = 0;
uint8_t packet[9];

uint8_t calculateChecksum(const uint8_t *buf) {
  uint8_t sum = 0;
  for (int i = 1; i < 8; ++i) sum += buf[i];
  return (~sum) + 1;
}

bool readPacket() {
  static uint8_t idx = 0;
  while (sensorSerial.available()) {
    uint8_t b = sensorSerial.read();
    if (idx == 0 && b != 0xFF) continue;
    packet[idx++] = b;
    if (idx == 9) {
      idx = 0;
      // print packet bytes
      Serial.print("Packet: ");
      for (int i = 0; i < 9; ++i) {
        Serial.print("0x");
        if (packet[i] < 16) Serial.print("0");
        Serial.print(packet[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      uint8_t cs = calculateChecksum(packet);
      Serial.print("Calculated checksum: 0x");
      if (cs < 16) Serial.print("0");
      Serial.print(cs, HEX);
      Serial.print(" | Packet checksum: 0x");
      if (packet[8] < 16) Serial.print("0");
      Serial.println(packet[8], HEX);

      if (cs == packet[8]) {
        Serial.println("Checksum OK");
        vocConcentration = ((uint16_t)packet[4] << 8) | packet[5];
        Serial.print("TVOC concentration: ");
        Serial.print(vocConcentration);
        Serial.println(" ug/m3");
        Serial.println("---------");
        return true;
      } else {
        Serial.println("Checksum ERROR");
      }
    }
  }
  return false;
}

void requestEvent() {
  // send just pVgas and TVOC
  String payload = String(sensor1.pVgas, 3) + "," + String(vocConcentration) + "\n";
  Wire.write(payload.c_str());
}

void receiveEvent(int howMany) {
  String msg;
  while (Wire.available()) {
    msg += char(Wire.read());
  }
  msg.trim();
  if (msg.equalsIgnoreCase("OK")) {
    ackReceived = true;
  }
}

void setup() {
  Serial.begin(115200);
  sensorSerial.begin(9600);
  analogReadResolution(10);
  // 2) Full-scale 0–3.3 V
  analogSetPinAttenuation(C1, ADC_11db);
  analogSetPinAttenuation(T1, ADC_11db);
  
  sensor1.pVcc = 3.3;
  sensor1.pVsup = 3.3;
  sensor1.pVref_set = 1.6;


  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  Serial.println("I2C Slave Ready");
}

void loop() {
  unsigned long now = millis();

  // periodic full debug
  if (now - lastSampleMillis >= SAMPLE_INTERVAL) {
    lastSampleMillis = now;

    sensor1.getIgas(GAS_AVG_SEC);
    sensor1.getTemp(TEMP_AVG_SEC);
    sensor1.getConc(sensor1.pT);

    // update TVOC if a packet is waiting
    while (readPacket()) {}

    Serial.print("Periodic sensor data - ");
    Serial.print(sensor1.convertT('C'));
    Serial.print(" C, Vgas: ");
    Serial.print(sensor1.pVgas);
    Serial.print(" V, Igas: ");
    Serial.print(sensor1.pInA);
    Serial.print(" nA, SO2: ");
    Serial.print(sensor1.pX);
    Serial.print(" ppb, TVOC: ");
    Serial.print(vocConcentration);
    Serial.println(" ug/m3");
  }

  // debug on acknowledgment
  if (ackReceived) {
    Serial.println("Master acknowledged with OK");
    // show raw TVOC packet details
    while (readPacket()) {}
    ackReceived = false;
  }
}
