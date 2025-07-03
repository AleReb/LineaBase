#include <ModbusMaster.h>
#include <SoftwareSerial.h>

// RS485 control pins
#define DE_PIN 14
#define RE_PIN 14

// SoftwareSerial pins (example for ESP32-C3)
#define RS485_RX 27
#define RS485_TX 15
#define SENSOR_POWER_PIN 4
// Initialize SoftwareSerial for RS485 communication
SoftwareSerial RS485Serial(RS485_RX, RS485_TX);   // (RX, TX)

// Create an instance of ModbusMaster for the sensor (slave address 1)
ModbusMaster node;

// RS485 transmission control functions
void preTransmission() {
  digitalWrite(DE_PIN, HIGH);
  digitalWrite(RE_PIN, HIGH);
}
void postTransmission() {
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);
}

// Define register addresses per manual (all values in hex as per PLC address)
// 0x1F4 (40501): Wind speed (0.01 m/s per count)
// 0x1F5 (40502): Wind strength (level index)
// 0x1F6 (40503): Wind direction (discrete: 0-7)
// 0x1F7 (40504): Wind direction (0-360°)
// 0x1F8 (40505): Humidity (0.1 %RH per count)
// 0x1F9 (40506): Temperature (0.1 °C per count, signed)
// 0x1FA (40507): Noise (0.1 dB per count)
// 0x1FB (40508): PM2.5 (1 µg/m³ per count)
// 0x1FC (40509): PM10 (1 µg/m³ per count)
// 0x1FD (40510): Atmospheric pressure (0.1 kPa per count)
// 0x1FE (40511): Illuminance (0-200000 Lux) high 16 byte (1 Lux per count)
// 0x1FF (40512): Illuminance (0-200000 Lux) low 16 byte (1 Lux per count)
// 0x200 (40513): Illuminance (0-200000 Lux) combined (100 Lux per count)
// 0x201 (40514): Rainfall (0.1 mm per count)
// 0x202 (40515): Reserve
// 0x203 (40516): Solar irradiance (1 W/m² per count)

#define REG_WIND_SPEED 0x1F4       // Wind speed register
#define REG_WIND_STRENGTH 0x1F5    // Wind strength register
#define REG_WIND_DIR_DISCRETE 0x1F6 // Wind direction (0-7) register
#define REG_WIND_DIR_DEGREE 0x1F7  // Wind direction (0-360°) register
#define REG_HUMIDITY 0x1F8         // Humidity register
#define REG_TEMPERATURE 0x1F9      // Temperature register
#define REG_NOISE 0x1FA            // Noise register
#define REG_PM25 0x1FB             // PM2.5 register
#define REG_PM10 0x1FC             // PM10 register
#define REG_PRESSURE 0x1FD         // Atmospheric pressure register
#define REG_ILLUMINANCE_HIGH 0x1FE // Illuminance high 16 byte register
#define REG_ILLUMINANCE_LOW 0x1FF  // Illuminance low 16 byte register
#define REG_ILLUMINANCE_COMBINED 0x200 // Illuminance combined register
#define REG_RAINFALL 0x201         // Rainfall register
#define REG_SOLAR_IRRADIANCE 0x203 // Solar irradiance register


void setup() {
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Serial to initialize

  // Set RS485 control pins as outputs and initialize in receive mode
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);

  // Start RS485 communication with the default 4800 bps
  RS485Serial.begin(4800);

  // Initialize the Modbus node (slave address 1)
  node.begin(1, RS485Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("Starting sensor read with all available parameters...");
}

void loop() {
  uint8_t result;

  // Read multiple registers to get all parameters
  // Start from 0x1F4 and read up to 0x203. This is 0x203 - 0x1F4 + 1 = 16 registers.
  // The ModbusMaster library's readHoldingRegisters function takes the starting address and the number of registers.
  // The response buffer will contain the values in the order of their addresses.
  result = node.readHoldingRegisters(REG_WIND_SPEED, 16); // Read 16 registers starting from 0x1F4

  if (result == node.ku8MBSuccess) {
    // Retrieve raw values from the response buffer
    uint16_t rawWindSpeed = node.getResponseBuffer(REG_WIND_SPEED - REG_WIND_SPEED);
    uint16_t rawWindStrength = node.getResponseBuffer(REG_WIND_STRENGTH - REG_WIND_SPEED);
    uint16_t rawWindDirDiscrete = node.getResponseBuffer(REG_WIND_DIR_DISCRETE - REG_WIND_SPEED);
    uint16_t rawWindDirDegree = node.getResponseBuffer(REG_WIND_DIR_DEGREE - REG_WIND_SPEED);
    uint16_t rawHumidity = node.getResponseBuffer(REG_HUMIDITY - REG_WIND_SPEED);
    uint16_t rawTemperature = node.getResponseBuffer(REG_TEMPERATURE - REG_WIND_SPEED);
    uint16_t rawNoise = node.getResponseBuffer(REG_NOISE - REG_WIND_SPEED);
    uint16_t rawPM25 = node.getResponseBuffer(REG_PM25 - REG_WIND_SPEED);
    uint16_t rawPM10 = node.getResponseBuffer(REG_PM10 - REG_WIND_SPEED);
    uint16_t rawPressure = node.getResponseBuffer(REG_PRESSURE - REG_WIND_SPEED);
    uint16_t rawIlluminanceHigh = node.getResponseBuffer(REG_ILLUMINANCE_HIGH - REG_WIND_SPEED);
    uint16_t rawIlluminanceLow = node.getResponseBuffer(REG_ILLUMINANCE_LOW - REG_WIND_SPEED);
    uint16_t rawIlluminanceCombined = node.getResponseBuffer(REG_ILLUMINANCE_COMBINED - REG_WIND_SPEED);
    uint16_t rawRainfall = node.getResponseBuffer(REG_RAINFALL - REG_WIND_SPEED);
    // Skipping 0x202 (Reserve register)
    uint16_t rawSolarIrradiance = node.getResponseBuffer(REG_SOLAR_IRRADIANCE - REG_WIND_SPEED);

    // Convert raw values using scaling factors as per manual
    float windSpeed = rawWindSpeed * 0.01;                              //// in m/s
    uint16_t windStrength = rawWindStrength;                             //   [cite_start]// as index value
    uint16_t windDirectionDiscrete = rawWindDirDiscrete;                //    [cite_start]// 0: north, 1: northeast, etc. [cite: 108]
    uint16_t windDirection = rawWindDirDegree;                             // [cite_start]// in degrees (0-360) [cite: 108]
    float humidity = rawHumidity * 0.1;                                    // [cite_start]// in %RH [cite: 108]
    float temperature = ((int16_t)rawTemperature) * 0.1;                  // [cite_start]// in °C (cast to signed to handle negative temperatures) [cite: 108]
    float noise = rawNoise * 0.1;                                          // [cite_start]// in dB [cite: 108]
    uint16_t pm25 = rawPM25;                                               // [cite_start]// in µg/m³ [cite: 108]
    uint16_t pm10 = rawPM10;                                                //[cite_start]// in µg/m³ [cite: 108]
    float pressure = rawPressure * 0.1;                                   // in kPa [cite: 108]

    // Illuminance: The manual provides 0x1FE (high 16 byte) and 0x1FF (low 16 byte) both at 1 Lux resolution,
    // and 0x200 (combined) at 100 Lux resolution.
    // For maximum precision, we combine the high and low 16-bit registers into a 32-bit value. [cite: 108]
   uint32_t actualIlluminance = ((uint32_t)rawIlluminanceHigh << 16) | rawIlluminanceLow;// [cite: 108]

    float rainfall = rawRainfall * 0.1;                                   // in mm [cite: 108]
    float solarIrradiance = rawSolarIrradiance * 1.0;                       // in W/m² (resolution 1 W/m²) [cite: 108]

    // Print sensor readings
    Serial.println("---- Sensor Readings ----");
    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s");

    Serial.print("Wind Strength: ");
    Serial.println(windStrength);

    Serial.print("Wind Direction (Discrete 0-7): ");
    Serial.println(windDirectionDiscrete);

    Serial.print("Wind Direction (Degrees, 0-360): ");
    Serial.print(windDirection);
    Serial.println("°");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %RH");

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Noise Level: ");
    Serial.print(noise);
    Serial.println(" dB");

    Serial.print("PM2.5: ");
    Serial.print(pm25);
    Serial.println(" µg/m³");

    Serial.print("PM10: ");
    Serial.print(pm10);
    Serial.println(" µg/m³");

    Serial.print("Atmospheric Pressure: ");
    Serial.print(pressure);
    Serial.println(" kPa");

    Serial.print("Illuminance: ");
    Serial.print(actualIlluminance); // Using the combined 32-bit value
    Serial.println(" Lux");

    Serial.print("Rainfall (Cumulative): ");
    Serial.print(rainfall);
    Serial.println(" mm");

    Serial.print("Solar Irradiance: ");
    Serial.print(solarIrradiance);
    Serial.println(" W/m²");

  } else {
    Serial.print("Modbus read error, code: 0x");
    Serial.println(result, HEX);
  }

  Serial.println("------------------------------\n");
  delay(5000); // Wait 5 seconds before the next read cycle
}
