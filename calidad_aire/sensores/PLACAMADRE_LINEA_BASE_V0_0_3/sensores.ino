void readAdcArraysold() {
  // ADS2 → adc2Raw[]
  adc2Raw[0] = ads2.readADC_SingleEnded(0);
  adc2Raw[1] = ads2.readADC_SingleEnded(1);
  adc2Raw[2] = ads2.readADC_SingleEnded(2);
  adc2Raw[3] = ads2.readADC_SingleEnded(3);

  // ADS1 → adc1Raw[]
  adc1Raw[0] = ads1.readADC_SingleEnded(0);
  adc1Raw[1] = ads1.readADC_SingleEnded(1);
  adc1Raw[2] = ads1.readADC_SingleEnded(2);
  adc1Raw[3] = ads1.readADC_SingleEnded(3);

  // ADS3 → adc3Raw[]
  adc3Raw[0] = ads3.readADC_SingleEnded(0);
  adc3Raw[1] = ads3.readADC_SingleEnded(1);
  adc3Raw[2] = ads3.readADC_SingleEnded(2);
  adc3Raw[3] = ads3.readADC_SingleEnded(3);
}
// funciones no bloqueantes en caso de error de sensores i2c
// --- I²C external sensor (TVOC + SO2) ---
// Helper: returns true si el dispositivo I2C responde en 'addr'
bool isDeviceConnected(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

//codigo revisando cada ADC
// Helper: attempt one ADS1115 read without blocking the bus if disconnected
// Try one ADS1115 read, return false if I²C fails
bool tryReadChannel(Adafruit_ADS1115 &adc, uint8_t address, uint8_t channel, float *out) {
  Wire.beginTransmission(address);
  if (Wire.endTransmission() != 0) {
    return false;  // device not responding
  }
  *out = static_cast<float>(adc.readADC_SingleEnded(channel));
  return true;
}

// Read ALL channels in one pass (blocks ≈1 ms por canal)
// Retries fallidos en la siguiente invocación
void readAllAdcChannelsOnce() {
  // ADS1 @ 0x48 → channels 0–3
  for (uint8_t ch = 0; ch < 4; ++ch) {
    if (!tryReadChannel(ads1, 0x48, ch, &adc1Raw[ch])) {
      Serial.printf("Error: ADS1 (0x48) ch %u failed\n", ch);
    }
  }

  // ADS2 @ 0x49 → channels 0–3
  for (uint8_t ch = 0; ch < 4; ++ch) {
    if (!tryReadChannel(ads2, 0x49, ch, &adc2Raw[ch])) {
      Serial.printf("Error: ADS2 (0x49) ch %u failed\n", ch);
    }
  }

  // ADS3 @ 0x4A → channels 0–1 (o 0–3 si los necesitas)
  for (uint8_t ch = 0; ch < 2; ++ch) {
    if (!tryReadChannel(ads3, 0x4A, ch, &adc3Raw[ch])) {
      Serial.printf("Error: ADS3 (0x4A) ch %u failed\n", ch);
    }
  }
}

void updateRawValues() {
  
  readAllAdcChannelsOnce();

  // Map array → globals
  adc2No2Op1Ch1Raw = adc2Raw[0];
  adc2No2Op2Ch2Raw = adc2Raw[1];
  adc2CoOp1Ch3Raw = adc2Raw[2];
  adc2CoOp2Ch4Raw = adc2Raw[3];

  adc1OxOp1Ch1Raw = adc1Raw[0];
  adc1OxOp2Ch2Raw = adc1Raw[1];
  adc1NoOp1Ch3Raw = adc1Raw[2];
  adc1NoOp2Ch4Raw = adc1Raw[3];

  adc3So2Op1Ch1Raw = adc3Raw[0];
  adc3So2Op2Ch2Raw = adc3Raw[1];
  // adc3XXOp1Ch3Raw = adc3Raw[2];
  // adc3XXOp2Ch4Raw = adc3Raw[3];
}

void printRawValues() {
  Serial.printf("ADC1 NO2  OP1/CH1 RAW: %.0f", adc2No2Op1Ch1Raw);
  Serial.printf(" ADC1 NO2  OP2/CH2 RAW: %.0f\n", adc2No2Op2Ch2Raw);
  Serial.printf("ADC1 CO   OP1/CH3 RAW: %.0f", adc2CoOp1Ch3Raw);
  Serial.printf(" ADC1 CO   OP2/CH4 RAW: %.0f\n", adc2CoOp2Ch4Raw);
  yield();

  Serial.printf("ADC2 OX   OP1/CH1 RAW: %.0f", adc1OxOp1Ch1Raw);
  Serial.printf(" ADC2 OX   OP2/CH2 RAW: %.0f\n", adc1OxOp2Ch2Raw);
  Serial.printf("ADC2 NO   OP1/CH3 RAW: %.0f", adc1NoOp1Ch3Raw);
  Serial.printf(" ADC2 NO   OP2/CH4 RAW: %.0f\n", adc1NoOp2Ch4Raw);
  yield();

  Serial.printf("ADC3 So2   OP1/CH1 RAW: %.0f", adc3So2Op1Ch1Raw);
  Serial.printf(" ADC3 So2   OP2/CH2 RAW: %.0f\n", adc3So2Op2Ch2Raw);
  // Serial.printf("ADC3 XX   OP1/CH3 RAW: %.0f", adc3XXOp1Ch3Raw);
  // Serial.printf(" ADC3 XX   OP2/CH4 RAW: %.0f\n", adc3XXOp2Ch4Raw);
  yield();
}
// modbus rs485
void preTransmission() {
  digitalWrite(DE_PIN, HIGH);
  digitalWrite(RE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);
}
void readWindSensor() {
  modbus.begin(1, rs485Serial);  // ID = 1 estación meteorológica

  uint8_t result = modbus.readHoldingRegisters(0x01F4, 16);
  if (result == modbus.ku8MBSuccess) {
    windSpeed = modbus.getResponseBuffer(0) * 0.01;
    windStrength = modbus.getResponseBuffer(1);
    windDirection = modbus.getResponseBuffer(3);
    windHumidity = modbus.getResponseBuffer(4) * 0.1;
    windTemperature = (int16_t)modbus.getResponseBuffer(5) * 0.1;
    windPressure = modbus.getResponseBuffer(9) * 0.1;
    windRainfall = modbus.getResponseBuffer(13) * 0.1;
    windSolarIrradiance = modbus.getResponseBuffer(15);

    Serial.printf("[WIND] Speed: %.2f m/s  Strength: %u  Dir: %u deg\n", windSpeed, windStrength, windDirection);
    Serial.printf("[WIND] Hum: %.1f %%  Temp: %.1f C  Pres: %.1f kPa  Rain: %.1f mm  Solar: %.1f W/m2\n",
                  windHumidity, windTemperature, windPressure, windRainfall, windSolarIrradiance);
    RS485METEO = true;
  } else {
    Serial.printf("Modbus error (wind): 0x%02X\n", result);
    RS485METEO = false;
  }
}

void readUVSensor() {
  modbus.begin(2, rs485Serial);  // ID = 2 sensor UV

  uint8_t result = modbus.readHoldingRegisters(0x0000, 1);
  if (result == modbus.ku8MBSuccess) {
    uvIntensity_mWcm2 = modbus.getResponseBuffer(0) * 0.01f;
    Serial.printf("[UV] Intensity: %.2f mW/cm²\n", uvIntensity_mWcm2);
    RS485UV = true;
  } else {
    Serial.printf("Modbus error (UV intensity): %u\n", result);
    RS485UV = false;
  }

  result = modbus.readHoldingRegisters(0x0001, 1);
  if (result == modbus.ku8MBSuccess) {
    uvIndex = modbus.getResponseBuffer(0);
    Serial.printf("[UV] Index: %u\n", uvIndex);
    RS485UV = true;
  } else {
    Serial.printf("Modbus error (UV index): %u\n", result);
    RS485UV = false;
  }
}
void I2CextEsp32C3() {
  i2cBytes = Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)32);
  if (i2cBytes == 0) {
    Serial.println("No I2C data received");
    I2CTVOCYSo2 = false;
  } else {
    // Clean payload (remove CR/LF)
    i2cBufLen = 0;
    while (Wire.available() && i2cBufLen < sizeof(i2cBuf) - 1) {
      char c = Wire.read();
      if (c != '\r' && c != '\n') {
        i2cBuf[i2cBufLen++] = c;
      }
      yield();
    }
    i2cBuf[i2cBufLen] = '\0';
    Serial.print("Clean I2C payload: ");
    Serial.println(i2cBuf);

    // Parse "<SO2_ppm>,<TVOC_ug/m3>"
    if (sscanf(i2cBuf, "%f,%hu", &so2Ppm, &tvocValue) == 2) {
      Serial.printf("Parsed SO2: %.3f raw  TVOC: %u ug/m3\n", so2Ppm, tvocValue);
      yield();
      I2CTVOCYSo2 = true;
    } else {
      Serial.println("I2C parsing error");
    }

    // Send ACK back
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("OK");
    Wire.endTransmission();
    Serial.println("Sent I2C acknowledgment: OK");
  }
}

void pmsReadSensor() {
  if (pmsSerial.available() >= 32) {
    for (uint8_t i = 0; i < 32; i++) {
      pmsBuffer[i] = pmsSerial.read();
      yield();
    }
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++) sum += pmsBuffer[i];
    uint16_t crc = (pmsBuffer[30] << 8) | pmsBuffer[31];
    if (sum == crc) {
      pm1 = (pmsBuffer[10] << 8) | pmsBuffer[11];
      pm25 = (pmsBuffer[12] << 8) | pmsBuffer[13];
      pm10 = (pmsBuffer[14] << 8) | pmsBuffer[15];
      pmsTemp = ((pmsBuffer[24] << 8) | pmsBuffer[25]) / 10.0;
      pmsHum = ((pmsBuffer[26] << 8) | pmsBuffer[27]) / 10.0;
      Serial.printf("PMS Temp: %.1f C  Hum: %.1f %%  PM1.0: %u ug/m3  PM2.5: %u ug/m3  PM10: %u ug/m3\n",
                    pmsTemp, pmsHum, pm1, pm25, pm10);
      yield();
      PLANTOWER = true;
    } else {
      Serial.println("PMS Error: checksum mismatch");
      PLANTOWER = false;
    }
  } else {
    Serial.println("PMS: no data");
  }
}

void co2ReadSensor() {
  co2Ppm = co2Sensor.getCO2();
  if (co2Sensor.errorCode == RESULT_OK)  // RESULT_OK is an alis for 1. Either can be used to confirm the response was OK.
  {
    co2Raw = co2Sensor.getCO2Raw();
    co2Custom = -0.674 * co2Raw + 36442;

    // concatenación en una sola String
    String output = "CO2_Internal: " + String(co2Ppm)
                    + " ppm  CO2_Raw: " + String(co2Raw, 2)
                    + "  CO2_Custom: " + String(co2Custom, 2) + " ppm";
    Serial.println(output);
    Co2 = true;

  } else {
    Serial.print("Response Code: ");
    Serial.println(co2Sensor.errorCode);  // Get the Error Code value
    Co2 = true;
  }
  yield();
}