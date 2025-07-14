/*
SD: timestamp(persona)fecha hora,BateriaEstacion,temperaturaEstacion,SEÑAL,adc2 CO raw OP1/CH3,CO rawOP2/ch4,adc2 N02 OP1/CH1 RAW,NO2 OP2/CH2,adc1 N0 OP1/CH3 RAW, NO OP2/CH4 RAW,adc1 OX OP1/CH1 RAW,OX OP2/CH2,PMS TEMP,PMS HUM,PMS 2.5,CO2_internal,CO2_RAW,CO2_CUSTOM,ESTACION TEMP,ESTACION HUM,ESTACION PRESION,VIENTO VELOCIDAD,VIENTO DIRECCION,LLUVIA ACUMULADA TOTAL,RADIACION TOTAL SOLAR,TOVC,SO2 RAW 
*/
// Placa Madre Rev. D

#define stationId 1
#define firmwareVersion 0.0.3

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 2014  // Set RX buffer to 2Kb
#define SerialAT Serial1

#include <esp_system.h>
#include "esp_adc_cal.h"  //better batt
#include "esp_task_wdt.h"
// --- Watchdog timeout (segundos) ---
const int WDT_TIMEOUT_S = 90;
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include "MHZ19.h"
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <Arduino.h>
#include <StreamDebugger.h>
#include <TinyGsmClient.h>
#include <U8g2lib.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define SD_CS_PIN 5  //para revision B es el 4 //el 5 revision D
//webserver rescate SD
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);
File uploadFile;
const char *ssid = "LINEABASE_EST01";
const char *password = "12345678";

// --- Transfer settings foto ---------------------------------------------------
#define CHUNK_SIZE 256
int RECEIVE_TIMEOUT = 45000;     // este timeoyt va a ser nas grande cuando lo intente por segunda vez solo para preubas
unsigned long lastByteTime = 0;  // To track last received byte time
bool sendAfterReceive = false;   // para reenviar foto
bool receiving = false;          //bool cuando esta recibiendo por serial
bool SavedSDafter = false;       // para asegurarse de que se guardo en la sd
File outFile;                    //tambien relacionado con el manejo de las fotos
String filename;                 //del archivo que se recepciona foto en la sd
int fileSize = 0;                //del string recibido para parsear el tama;o del archivo foto
int bytesReceived = 0;           //del manejo foto
String PHOTO_PATH = "";          ///////////-------------------------- // Ensure PHOTO_PATH is always initialized para mandar la foto por gsm
bool hasRetried = false;         //del manejo de fotos
bool photosenderOK = false;      //recibi ready de la rpi
//---conectividad
const char *WEBHOOK_URL = "https://webhook.site/13031b43-9a7b-486e-ac33-641ecf0af84e/insertarMedicion?idsSensores=121,121,121,122,122,122,151,151,151,151,151,151,151,124,124,125,125,126,126,127,127,155,145,152,128,129&idsVariables=31,32,33,8,3,6,3,6,29,30,5,17,13,18,28,19,27,20,26,21,25,34,24,3,4,15&valores=";  //pruebas exitosas
const char *NGROK_URL = "https://hermit-harmless-wildly.ngrok-free.app/agregarImagen";                                                                                                                                                                                                                                   //pruebas exitosas
const char *BASE_SERVER_URL = "http://api-sensores.cmasccp.cl/insertarMedicion?idsSensores=121,121,121,122,122,122,151,151,151,151,151,151,151,124,124,125,125,126,126,127,127,155,145,152,128,129&idsVariables=31,32,33,8,3,6,3,6,29,30,5,17,13,18,28,19,27,20,26,21,25,34,24,3,4,15&valores=";                         //1,576,96,2,3,4,5,6,7,8,9,10,66,11,12,13,14,15,16,17,18,22,23,19,20,21"

// Configuración de tiempos
#define uS_TO_S_FACTOR 1000000ULL  // Factor de conversión de microsegundos a segundos
#define TIME_TO_SLEEP 30           // Tiempo de sleep en segundos
#define UART_BAUD 115200

//---------------uart modem
#define MODEM_RX_PIN 17
#define MODEM_TX_PIN 16
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
TinyGsmClient client(modem);

// --- GPRS & Webhook config -----------------------------------------------
const char *APN = "gigsky-02";
const char *GPRS_USER = "";
const char *GPRS_PASS = "";
String networkOperator;
String networkTech;
String signalQuality;
String registrationStatus;
String httpReadData;
String lastPostedID;

// —— Pin definitions ——
const int SENSOR_POWER_PIN = 4;
const int CAM_POWER_PIN = 25;
const int MODEM_POWER_PIN = 13;

// bat formulae Divider resistor values (in kilo-ohms)
// --- Pin y divisor ---
#define BATT_SENSE_PIN 36  // ADC1_CH0 en GPIO36
const float R1 = 470.0;    // top resistor (kΩ)
const float R2 = 100.0;    // bottom resistor (kΩ)
// --- Parámetros ADC ---
const uint32_t DEFAULT_VREF = 1100;  // mV, ajústalo si mides otra cosa
esp_adc_cal_characteristics_t adc_chars;

// --- Serial port pins ---------------

// CAMARA
#define FILE_RX_PIN 35
#define FILE_TX_PIN 26
SoftwareSerial fileSerial(FILE_RX_PIN, FILE_TX_PIN);
#define CAMBAUD 31250
// —— ADS1115 instances ——
Adafruit_ADS1115 ads1;  // address 0x48
Adafruit_ADS1115 ads2;  // address 0x49

// —— MH-Z19 CO₂ sensor ——
#define MHZ19_RX_PIN 34
#define MHZ19_TX_PIN 32
SoftwareSerial co2Serial(MHZ19_RX_PIN, MHZ19_TX_PIN);
MHZ19 co2Sensor;

// —— PMS5003 particulate sensor ——
const int PMS_RX_PIN = 39;
const int PMS_TX_PIN = 33;
SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN);
uint8_t pmsBuffer[32];

// —— Modbus wind sensor ——
const int DE_PIN = 14;
const int RE_PIN = 14;
const int RS485_RX_PIN = 27;
const int RS485_TX_PIN = 15;
SoftwareSerial rs485Serial(RS485_RX_PIN, RS485_TX_PIN);
ModbusMaster windNode;

// --- Display & RTC & rgb -----------------------
U8G2_SH1107_SEEED_128X128_F_HW_I2C display(U8G2_R0);
RTC_DS3231 rtc;

const int rgbPowerPin = GPIO_NUM_12;
Adafruit_NeoPixel pixels(1, 2, NEO_GRB + NEO_KHZ800);

// —— Global variables for readings ——
String fileCSV;
// ADC readings
float adc1Raw[4];
float adc2Raw[4];

// CO₂ readings
double co2Ppm;
double co2Raw;
double co2Custom;

// PMS5003 readings
uint16_t pm1, pm25, pm10;
float pmsTemp, pmsHum;

// Wind sensor readings
uint8_t modbusResult;
float windSpeed;
uint16_t windStrength, windDirection;
float windHumidity, windTemperature, windPressure;
float windRainfall, windSolarIrradiance;

// I²C payload buffer
const uint8_t I2C_SLAVE_ADDR = 0x08;
char i2cBuf[33];
uint8_t i2cBytes, i2cBufLen;
float so2Ppm;
uint16_t tvocValue;
// factores internos, temperatura rtc y bateria
float stationTemperature;
float batteryLevel;
float signalValue;  ////////////es del modem
// —— Timing globals ——
unsigned long prevLoopMillis = 0;
const unsigned long LOOP_INTERVAL_MS = 1000 * 20;  //registradatos

unsigned long SaveMillis = 0;
const unsigned long SAVE_INTERVAL_MS = 1000 * 60;  //guarda datos

unsigned long SaveFotoMillis = 0;
const unsigned long FOTO_INTERVAL_MS = 1000 * 60 * 60 * 3;  //guarda y manda foto ca 3 hora

unsigned long SendMillis = 0;
const unsigned long SEND_INTERVAL_MS = 1000 * 60 * 5;  //manda datos

// --- State variables ------------------------------------------------------
const unsigned long networkInterval = 60000 * 15;  // revisa si esta conectado, puede que tope, revisar
unsigned long lastNetworkUpdate = 0;               // chequear conecxion
const unsigned long sendInterval = 60000 * 60;     // 100 min 6000000 mandar por hora
unsigned long lastDataSend = 0;                    //mandar
//----------Global bools for control-----------------
bool clientConnected = false;  // Variable para indicar si hay una conexión activa WIFI
const bool DEBUG = true;       // Debug flag para seriales del wifi
bool requestedfoto = false;    // se solicito foto control de estructura maquina de estados
bool savedSD = false;          //guardando evita las otras funciones mauqina de estado
bool sending = false;          //mandando por gsm control de estado
bool leyendoDatos = false;     // leyendo sensores control de estados
int strikes = 0;               //reintentos de la camara
bool FirstRun = true;

void setup() {
  // Serial console
  muxCycleLeds();
  Serial.begin(115200);
  while (!Serial) { delay(10); }  //
  print_wakeup_reason();
  // Power on sensors
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  pinMode(MODEM_POWER_PIN, OUTPUT);
  digitalWrite(MODEM_POWER_PIN, HIGH);

  pinMode(CAM_POWER_PIN, OUTPUT);
  digitalWrite(CAM_POWER_PIN, HIGH);
  // gsm celular
  // USB-Serial and AT-Serial
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  modem.restart();
  modem.init();
  testSIM();
  if (!connectToNetwork()) {
    Serial.println("Network connect failed");
  }
  updateNetworkInfo();
  // Initialize I²C
  Wire.begin();

  // Initialize ADS1115
  if (!ads1.begin(0x48)) {
    Serial.println("Error: ADS1 init failed");
  } else {
    Serial.println("ADS1 init OK");
  }
  if (!ads2.begin(0x49)) {
    Serial.println("Error: ADS2 init failed");
  } else {
    Serial.println("ADS2 init OK");
  }
  ads1.setGain(GAIN_TWO);
  ads2.setGain(GAIN_TWO);

  // Initialize MH-Z19
  co2Serial.begin(9600);
  co2Sensor.begin(co2Serial);
  co2Sensor.autoCalibration(false);

  // Initialize PMS5003
  pmsSerial.begin(9600);
  // File-transfer serial camara
  fileSerial.begin(CAMBAUD);
  // analog stuff

  analogSetPinAttenuation(BATT_SENSE_PIN, ADC_11db);
  analogSetWidth(ADC_WIDTH_BIT_12);

  // *** Calibración IDF ***
  esp_adc_cal_characterize(
    ADC_UNIT_1,
    ADC_ATTEN_DB_11,
    ADC_WIDTH_BIT_12,
    DEFAULT_VREF,
    &adc_chars);

  // Initialize RS-485 Modbus
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);
  rs485Serial.begin(4800);
  windNode.begin(1, rs485Serial);
  windNode.preTransmission(preTransmission);
  windNode.postTransmission(postTransmission);

  // RTC
  if (!rtc.begin()) {
    Serial.println("RTC init failed");
    display.clearBuffer();
    display.drawStr(0, 24, "RTC FAIL");
    display.sendBuffer();
  } else {
    Serial.println("RTC OK");
    checkAndUpdateRTC(rtc);
  }
  SPI.begin();  // <— ensures SCK/MISO/MOSI are driven
  // SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed");
    display.clearBuffer();
    display.drawStr(0, 24, "SD FAIL");
    display.sendBuffer();
    delay(1000);
  } else {
    Serial.println("SD OK");
    ensureDataFileExists();
  }
  // Ensure "fotos" directory exists
  if (!SD.exists("/fotos")) {
    Serial.println("Directory /fotos EXIST");
    if (SD.mkdir("/fotos")) {
      Serial.println("Directory /fotos created");
    } else {
      Serial.println("Failed to create /fotos directory");
    }
  }
  Serial.println("=== All sensors initialized ===");
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Setup HTTP server routes
  server.on("/", HTTP_GET, handleList);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/view", HTTP_GET, handleView);
  server.on("/delete", HTTP_GET, handleDelete);
  server.on("/rename", HTTP_GET, handleRename);
  server.on("/mkdir", HTTP_GET, handleMkdir);
  server.on(
    "/upload", HTTP_POST, []() {
      server.send(200, "text/plain", "Upload complete");
    },
    handleUpload);

  // Enable CORS for better Windows browser compatibility
  server.enableCORS(true);

  server.begin();
  Serial.println("=== All WIFI ok ===");
  // 1) Inicializa el Task Watchdog
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  // --- Alimenta al watchdog antes de que expire ---
  if (FirstRun == true) {
    if (digitalRead(CAM_POWER_PIN) == LOW) {
      UpDATA();
    }
   FirstRun = false;
  }
  esp_task_wdt_reset();
  handleSerialCommands();
  if (leyendoDatos == false && savedSD == false && sending == false) {
    clientConnected = (WiFi.softAPgetStationNum() > 0);
  }
  unsigned long now = millis();
  if (clientConnected) {
    // digitalWrite(rgbPowerPin, HIGH);
    // pixels.setPixelColor(0, pixels.Color(0, 15, 15));
    // pixels.show();
    server.handleClient();
    // ensureAP();
    yield();
  } else {
    checkReadyTimeout();
   // readModemResponses(); //funcion vieja
     readModemResponses2();
    // —— Main loop interval ——
    if (!receiving && fileSerial.available()) {
      readHeader();
      yield();
    }
    // 2) Si estamos recibiendo, proceso la recepción y salgo
    else if (receiving) {
      processReception();
      return;
    }
    if (requestedfoto == false && savedSD == false && sending == false && receiving == false) {
      leyendoDatos = true;
      if (now - prevLoopMillis >= LOOP_INTERVAL_MS) {
        prevLoopMillis = now;
        //battery
        Serial.print("Operador: " + networkOperator);
        Serial.print(" Tecnología: " + networkTech);
        Serial.print(" señal CSQ: " + signalQuality);
        Serial.println(" Conectado: " + registrationStatus);


        uint32_t raw = analogRead(BATT_SENSE_PIN);
        uint32_t vSense_mV = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

        float vSense = vSense_mV / 1000.0f;
        batteryLevel = vSense * ((R1 + R2) / R2);

        Serial.printf(
          "Raw: %4d | V_sense: %.3f V | V_bat: %.3f V\n",
          raw, vSense, batteryLevel);
        for (uint8_t ch = 0; ch < 4; ch++) {
          adc1Raw[ch] = ads1.readADC_SingleEnded(ch);
          adc2Raw[ch] = ads2.readADC_SingleEnded(ch);
        }
        Serial.print("ADC1:");
        for (uint8_t ch = 0; ch < 4; ch++) {
          Serial.printf(" CH%d %.4f RAW", ch + 1, adc1Raw[ch]);
          yield();
        }
        Serial.println();
        Serial.print("ADC2:");
        for (uint8_t ch = 0; ch < 4; ch++) {
          Serial.printf(" CH%d %.4f RAW", ch + 1, adc2Raw[ch]);
          yield();
        }
        Serial.println();

        // 2) Read MH-Z19 CO2
        co2Ppm = co2Sensor.getCO2();
        co2Raw = co2Sensor.getCO2Raw();
        co2Custom = -0.674 * co2Raw + 36442;
        Serial.printf("CO2_Internal: %d ppm  CO2_Raw: %.2f  CO2_Custom: %.2f ppm\n",
                      co2Ppm, co2Raw, co2Custom);
        yield();
        // 3) Read PMS5003
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
          } else {
            Serial.println("PMS Error: checksum mismatch");
          }
        } else {
          Serial.println("PMS: no data");
        }

        // 4) Read wind sensor via Modbus
        modbusResult = windNode.readHoldingRegisters(0x1F4, 16);
        if (modbusResult == windNode.ku8MBSuccess) {
          windSpeed = windNode.getResponseBuffer(0) * 0.01;
          windStrength = windNode.getResponseBuffer(1);
          windDirection = windNode.getResponseBuffer(3);
          windHumidity = windNode.getResponseBuffer(4) * 0.1;
          windTemperature = int16_t(windNode.getResponseBuffer(5)) * 0.1;
          windPressure = windNode.getResponseBuffer(9) * 0.1;
          windRainfall = windNode.getResponseBuffer(13) * 0.1;
          windSolarIrradiance = windNode.getResponseBuffer(15);
          Serial.printf("Wind Speed: %.2f m/s  Strength: %u  Dir: %u deg\n", windSpeed, windStrength, windDirection);
          Serial.printf("Humidity: %.1f %%  Temp: %.1f C  Pressure: %.1f kPa  Rainfall: %.1f mm  Solar: %.1f W/m2\n",
                        windHumidity, windTemperature, windPressure, windRainfall, windSolarIrradiance);
          yield();
        } else {
          Serial.printf("Modbus error: 0x%02X\n", modbusResult);
        }
        i2cBytes = Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)32);
        if (i2cBytes == 0) {
          Serial.println("No I2C data received");
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
          } else {
            Serial.println("I2C parsing error");
          }

          // Send ACK back
          Wire.beginTransmission(I2C_SLAVE_ADDR);
          Wire.write("OK");
          Wire.endTransmission();
          Serial.println("Sent I2C acknowledgment: OK");

          stationTemperature = rtc.getTemperature();
          Serial.print("Station Temperature (RTC): ");
          Serial.print(stationTemperature, 2);
          Serial.println(" °C");
          if (photosenderOK == true) {
            Serial.println("request foto: OK Strikes: " + String(strikes));

          } else {
            Serial.println("request foto: NO , no recibi mensaje de la cam");
          }

          Serial.println("-----------------------");
          leyendoDatos = false;
        }
        yield();
      }
    }
    //send foto after recibed foto now at start
    if (sendAfterReceive && SavedSDafter) {
      Serial.println("Sending image after receive...");
      sendImageWebhook();
      sendAfterReceive = false;
      SavedSDafter = false;
      digitalWrite(CAM_POWER_PIN, LOW);
    }

    // —— SD save interval ——
    if (now - SaveMillis >= SAVE_INTERVAL_MS && requestedfoto == false) {
      SaveMillis = now;
      yield();
      savedSD = true;
      Serial.println("guardando SD");
      //checkTime();
      blinkGreenLed();
      delay(100);
      writeDataToCSV();
      blinkGreenLed();
      delay(100);
      Serial.println("-----------------------");
      savedSD = false;
    }
    // —— CAM save and Send interval ——
    if (now - SaveFotoMillis >= FOTO_INTERVAL_MS && photosenderOK == true) {
      yield();
      SaveFotoMillis = now;
      Serial.println("aqui va la funcion Sacar foto y mandar");
      // requestedfoto = true;
      // fileSerial.println("foto");
      digitalWrite(CAM_POWER_PIN, HIGH);
      blinkBlueLed();
      yield();
      // sendAfterReceive = true;  // envío tras recibir
    }

    // —— SIGNAL CHECK interval ——
    if (now - lastNetworkUpdate >= networkInterval) {
      lastNetworkUpdate = now;
      sending = true;
      updateNetworkInfo();
      sending = false;
      yield();
    }
    // —— DATA SEND interval ——
    if (now - SendMillis >= SEND_INTERVAL_MS && requestedfoto == false) {
      yield();
      SendMillis = now;
      sending = true;
      Serial.println("mandando datos");
      // sendImageWebhook();  //directo tipo foto
      UpDATA();
      Serial.println("-----------------------");
      delay(50);
      sending = false;
      yield();
    }
  }
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
