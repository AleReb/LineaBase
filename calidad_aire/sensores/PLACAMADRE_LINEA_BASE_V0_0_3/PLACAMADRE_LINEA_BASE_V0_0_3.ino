/*
SD: timestamp(persona)fecha hora,BateriaEstacion,temperaturaEstacion,SEÑAL,adc2 CO raw OP1/CH3,CO rawOP2/ch4,adc2 N02 OP1/CH1 RAW,NO2 OP2/CH2,adc1 N0 OP1/CH3 RAW, NO OP2/CH4 RAW,adc1 OX OP1/CH1 RAW,OX OP2/CH2,PMS TEMP,PMS HUM,PMS 2.5,CO2_internal,CO2_RAW,CO2_CUSTOM,ESTACION TEMP,ESTACION HUM,ESTACION PRESION,VIENTO VELOCIDAD,VIENTO DIRECCION,LLUVIA ACUMULADA TOTAL,RADIACION TOTAL SOLAR,TOVC,SO2 RAW 
/*
 * Sistema de Monitoreo Ambiental (Placa Madre Rev. D)
 *
 * Autor: Alejandro Rebolledo
 * Revisión: 0.3
 * Fecha: 30 de julio del 2025
 *
 * Descripción general:
 * Este firmware implementa un sistema integrado para monitorear múltiples parámetros ambientales y climáticos.
 * Se encarga de adquirir datos desde diversos sensores (ADS1115, MH-Z19, PMS5003, sensores Modbus, TVOC, SO2, UV, etc.),
 * guardarlos en una tarjeta SD en formato CSV con timestamp, batería, calidad de señal y valores brutos de los sensores.
 * Además, envía datos periódicamente a través del módem GSM/LTE (SIM7600) hacia un servidor remoto.
 * El firmware gestiona también la captura, almacenamiento y envío periódico de imágenes capturadas mediante una cámara externa (interfaz serie).
 *
 * Características Principales:
 * - Lectura periódica y simultánea de múltiples sensores analógicos y digitales.
 * - Almacenamiento seguro en tarjeta SD con gestión robusta de archivos y directorios.
 * - Envío automático y periódico de datos al servidor remoto mediante protocolo HTTP usando módem SIM7600.
 * - Captura periódica y transmisión robusta de imágenes mediante comunicación UART con cámara externa.
 * - Uso eficiente de Watchdog (WDT) para asegurar estabilidad y auto-recuperación en caso de fallos.
 * - Control y monitoreo remoto del estado de la estación (batería, señal celular, calidad de sensores).
 * - Indicaciones visuales del estado del sistema mediante LED RGB.
 *
 * Estructura del código:
 * - Inicialización y verificación robusta de hardware y sensores.
 * - Manejo estructurado de máquina de estados para captura y envío de fotos.
 * - Verificación periódica y robusta de conectividad celular con reinicio automático del módem ante fallos persistentes.
 * - Optimización de consumo energético mediante control selectivo de encendido y apagado de periféricos.
 * - Gestión robusta de errores con almacenamiento en log dedicado en tarjeta SD.
 *
 * Consideraciones adicionales:
 * Este sistema está diseñado para monitoreo continuo en campo, garantizando estabilidad operativa y eficiencia energética.
 * Se recomienda verificar periódicamente el estado de sensores, almacenamiento SD y calidad de señal celular en terreno.
 */
#define stationId 1
#define firmwareVersion 0.0.3L

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
//#include <StreamDebugger.h> //depreciado
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
String PHOTO_PATH = "";          // Ensure PHOTO_PATH is always initialized para mandar la foto por gsm
bool hasRetried = false;         //del manejo de fotos
bool photosenderOK = false;      //recibi ready de la rpi

//---conectividad
const char *WEBHOOK_URL = "https://webhook.site/c9333b6f-f640-4e3b-8dce-914a17b553c7";                                                                                                                                                                                                                                                     //direccion de testeo
const char *NGROK_URL = "https://hermit-harmless-wildly.ngrok-free.app/agregarImagen";                                                                                                                                                                                                                                                     //pruebas exitosas
const char *BASE_SERVER_URL = "http://api-sensores.cmasccp.cl/insertarMedicion?idsSensores=121,121,121,122,122,122,122,151,151,151,151,151,151,151,124,124,125,125,126,126,127,127,158,158,159,159,155,145,156,152,128,129&idsVariables=31,32,33,8,9,3,6,3,6,29,30,5,17,13,18,28,19,27,20,26,21,25,37,38,39,40,34,24,35,3,4,15&valores=";  //1,576,96,2,3,4,5,6,7,8,9,10,66,11,12,13,14,15,16,17,18,22,23,19,20,21"

#define uS_TO_S_FACTOR 1000000ULL  // Factor de conversión de microsegundos a segundos
#define TIME_TO_SLEEP 30           // Tiempo de sleep en segundos
#define UART_BAUD 115200

//---------------uart modem
#define MODEM_RX_PIN 17
#define MODEM_TX_PIN 16

//StreamDebugger debugger(SerialAT, Serial);//eliminamos debugg
//TinyGsm modem(debugger); activamos el debug
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);


// --- GPRS & Webhook config -----------------------------------------------
const char *APN = "gigsky-02";
const char *GPRS_USER = "";
const char *GPRS_PASS = "";
String networkOperator;
String networkTech;
String signalQuality;
String registrationStatus;
String lastPostedID;
//modem helpers
bool httpSessionActive = false;
String lastAtCommand;
String httpReadData;

// — Máximo de strikes antes de reiniciar el módem
const int MAX_MODEM_STRIKES = 10;
int ModemStrikes = 0;

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
Adafruit_ADS1115 ads3;  // address 0x4A
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
ModbusMaster modbus;

// --- RTC & rgb -----------------------
U8G2_SH1107_SEEED_128X128_F_HW_I2C display(U8G2_R0);
RTC_DS3231 rtc;

#define RGB_PIN 2     // Pin where your NeoPixel is connected
#define NUM_PIXELS 1  // Número de LEDs en la tira

const int rgbPowerPin = GPIO_NUM_12;
Adafruit_NeoPixel pixels(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// —— Global variables for readings ——
String fileCSV;      //ruta del csv
String logFilePath;  // ruta del log
// ADC readings
float adc1Raw[4];
float adc2Raw[4];
float adc3Raw[4];
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
// UV
float uvIntensity_mWcm2 = 0.0;
uint16_t uvIndex = 0;

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
const unsigned long FOTO_INTERVAL_MS = 1010 * 60 * 60 * 3;  //guarda y manda foto ca 3 hora ligero desfase para no topar con el envio de datos cad 5 minutos

unsigned long SendMillis = 0;
const unsigned long SEND_INTERVAL_MS = 1000 * 60 * 5;  //manda datos

// --- State variables ------------------------------------------------------
const unsigned long networkInterval = 60000 * 30;  // revisa si esta conectado, puede que tope, revisar
unsigned long lastNetworkUpdate = 0;               // chequear conecxion

//----------Global bools for control-----------------
bool clientConnected = false;  // Variable para indicar si hay una conexión activa WIFI
const bool DEBUG = true;       // Debug flag para seriales del wifi
bool requestedfoto = false;    // se solicito foto control de estructura maquina de estados
bool savedSD = false;          //guardando evita las otras funciones mauqina de estado
bool sending = false;          //mandando por gsm control de estado
bool leyendoDatos = false;     // leyendo sensores control de estados
int strikes = 0;               //reintentos de la camara
bool FirstRun = true;
bool SDOK = false;
bool RTCOK = false;
bool ADC1 = false;
bool ADC2 = false;
bool ADC3 = false;
bool RS485METEO = false;
bool RS485UV = false;
bool I2CTVOCYSo2 = false;
bool PLANTOWER = false;
bool Co2 = false;
// --- Global raw sensor values ---
// ADC1 channels
float adc1OxOp1Ch1Raw;
float adc1OxOp2Ch2Raw;
float adc1NoOp1Ch3Raw;
float adc1NoOp2Ch4Raw;
// ADC2 channels
float adc2No2Op1Ch1Raw;
float adc2No2Op2Ch2Raw;
float adc2CoOp1Ch3Raw;
float adc2CoOp2Ch4Raw;
// ADC3 channels
float adc3So2Op1Ch1Raw;
float adc3So2Op2Ch2Raw;
float adc3XXOp1Ch3Raw;
float adc3XXOp2Ch4Raw;


void setup() {
  // Serial console
  muxCycleLeds();
  Serial.begin(115200);
  while (!Serial) { delay(10); }  //
  print_wakeup_reason();
  // Power on sensors
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  pinMode(CAM_POWER_PIN, OUTPUT);
  // digitalWrite(CAM_POWER_PIN, HIGH); // prendo la camara en el primer loop
  //MODEM
  pinMode(MODEM_POWER_PIN, OUTPUT);
  digitalWrite(MODEM_POWER_PIN, LOW);  //high para prender modem

  // Initialize I²C
  Wire.begin();
  delay(200);
  muxCycleLeds();

  // Initialize ADS1115
  if (!ads1.begin(0x48)) {
    Serial.println("Error: ADS1 init failed");
  } else {
    Serial.println("ADS1 init OK");
    ADC1 = true;
  }
  if (!ads2.begin(0x49)) {
    Serial.println("Error: ADS2 init failed");
  } else {
    Serial.println("ADS2 init OK");
    ADC2 = true;
  }
  if (!ads3.begin(0x4A)) {
    Serial.println("Error: ADS3 init failed");
  } else {
    Serial.println("ADS3 init OK");
    ADC3 = true;
  }
  ads1.setGain(GAIN_TWO);
  ads2.setGain(GAIN_TWO);
  ads3.setGain(GAIN_TWO);
  // Initialize MH-Z19
  co2Serial.begin(9600);
  co2Sensor.begin(co2Serial);
  co2Sensor.autoCalibration(false);
  // co2Sensor.printCommunication();  // Error Codes are also included here if found (mainly for debugging/interest)
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
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);

  // RTC
  if (!rtc.begin()) {
    Serial.println("RTC init failed");
    display.clearBuffer();
    display.drawStr(0, 24, "RTC FAIL");
    display.sendBuffer();
  } else {
    Serial.println("RTC OK");
    RTCOK = true;
    checkAndUpdateRTC(rtc);
  }
  SPI.begin();  // <— ensures SCK/MISO/MOSI are driven
  // SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed");
    display.clearBuffer();
    display.drawStr(0, 24, "SD FAIL");
    display.sendBuffer();
    muxRedLed();
    delay(5000);
    muxOffLed();
  } else {
    Serial.println("SD OK");
    SDOK = true;
   
    ensureDataFileExists();
    ensureLogFileExists();

    // Ensure "fotos" directory exists
    if (!SD.exists("/fotos")) {
      Serial.println("Directory /fotos EXIST");
      if (SD.mkdir("/fotos")) {
        Serial.println("Directory /fotos created");
      } else {
        Serial.println("Failed to create /fotos directory");
      }
    }

 Serial.print(" - ");
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD, "/", 0);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));

  }

  Serial.println("=== All sensors initialized ===");
  /*
  //wifi elminado por consumo  
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
  */
  // gsm celular
  // USB-Serial and AT-Serial
  delay(500);
  blinkBlueLed();
  digitalWrite(MODEM_POWER_PIN, HIGH);  //high para prender modem
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  modem.restart();
  modem.init();
  // DESCARTA URCs de arranque
  delay(200);
  while (modem.stream.available()) {
    modem.stream.readStringUntil('\n');
  }
  testSIM();
  if (!connectToNetwork()) {
    Serial.println("Network connect failed");
  }
  modemRTC();

  if (SDOK
      && RTCOK
      && ADC1
      && ADC2
      && ADC3) {
    muxGreenLed();
    delay(2000);
    muxOffLed();
  } else if (SDOK && !(RTCOK && ADC1 && ADC2 && ADC3)) {
    muxYellowLed();
    delay(5000);
    muxOffLed();
  }
  // LAST Inicializa el Task Watchdog siempre al final porque el modem puede bloquearse y entrar en loop de reinicio
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  handlePowerCycleModem();  //maquina de estados para reiniciar el modem en caso de muchos problemas de envio
  unsigned long now = millis();
  // --- Alimenta al watchdog antes de que expire ---
  esp_task_wdt_reset();    //reset watchdog
  handleSerialCommands();  //serialcomands para debugear revisar pestaña SerialComands
  readModemResponses();    //parsea los mensajes del MODEM Revisar pestaña gsm

  if (FirstRun == true) {
    updateNetworkInfo();
    digitalWrite(CAM_POWER_PIN, HIGH);  //prender la camara
    Serial.println("Prendida la camara");
    FirstRun = false;
  }
  if (ModemStrikes >= MAX_MODEM_STRIKES) {
    beginPowerCycleModem();
  }
  // if (leyendoDatos == false && savedSD == false && sending == false) {  //funcion vieja si vuelve el wifi
  //   // clientConnected = (WiFi.softAPgetStationNum() > 0); //desactivado por problemas de ADC
  // }
  // if (clientConnected) {
  //   //esta funciuon no se implementara en esta version por problemas de consumo
  //   // digitalWrite(rgbPowerPin, HIGH);
  //   // pixels.setPixelColor(0, pixels.Color(0, 15, 15));
  //   // pixels.show();
  //   //server.handleClient(); //desactivado
  //   // ensureAP();
  //   yield();
  // } else {

  /////ahora esto es main
  checkReadyTimeout();  //timeout de la camara la apaga si o si en 3 minutos
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

    if (sendAfterReceive && SavedSDafter) {
      Serial.println("Sending image after receive...");
      sendImageWebhook();
      sendAfterReceive = false;
      SavedSDafter = false;
    }

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
      Serial.println("V bateria: " + String(batteryLevel));

      updateRawValues();  // populate your globals
      printRawValues();   // dump to serial

      // 2) Read MH-Z19 CO2
      co2ReadSensor();
      // 3) Read PMS5003
      pmsReadSensor();
      // 4) Read wind sensor via Modbus
      delay(30);
      readWindSensor();  // ID 1
      delay(30);
      readUVSensor();  // ID 2
                       // i2c EXTERNO VOC
                       // --- I²C external sensor (TVOC + SO2) ---
      if (isDeviceConnected(I2C_SLAVE_ADDR)) {
        I2CextEsp32C3();  //se llama la funcion solo si el helper es true
      } else {
        I2CTVOCYSo2 = false;
        Serial.println("Error: I2C slave not responding");
      }


      stationTemperature = rtc.getTemperature();
      Serial.print("Station Temperature (RTC): ");
      Serial.print(stationTemperature, 2);
      Serial.println(" °C");

      if (photosenderOK == true) {
        Serial.println("request foto: OK Strikes: " + String(strikes));
      } else {
        Serial.println("request foto: NO , no recibi mensaje de la cam");
      }
      Serial.println("Strikes de transmision en httpinit: " + String(ModemStrikes));
      Serial.println("-----------------------");
      if (SDOK
          && RTCOK
          && ADC1
          && ADC2
          && ADC3
          && RS485METEO
          && RS485UV
          && I2CTVOCYSo2
          && PLANTOWER
          && Co2) {
        Serial.println("All OK");
        blinkGreenLed();
      } else {
        blinkYellowLed();
      }
      leyendoDatos = false;
    }
    yield();
  }

  // }///////// del antiguo wifi

  // —— SD save interval ——
  if (now - SaveMillis >= SAVE_INTERVAL_MS && requestedfoto == false) {
    SaveMillis = now;
    yield();
    savedSD = true;
    Serial.println("guardando SD");
    writeDataToCSV();
    Serial.println("-----------------------");
    savedSD = false;
  }

  // —— CAM save and Send interval ——
  if (now - SaveFotoMillis >= FOTO_INTERVAL_MS && photosenderOK == true) {
    yield();
    SaveFotoMillis = now;
    Serial.println("aqui va la funcion Sacar foto y mandar");
    requestedfoto = true;
    digitalWrite(CAM_POWER_PIN, HIGH);  // fileSerial.println("foto"); solo prendo la camara ya se espera el mensaje y el retry
    blinkOrangeLed();                   // naranja
    blinkOrangeLed();                   // naranja
    blinkOrangeLed();                   // naranja
    yield();
    sendAfterReceive = true;  // envío tras recibir
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
    //funcion mandar datos
    yield();
    SendMillis = now;
    sending = true;
    Serial.println("mandando datos");
    UpDATA();
    Serial.println("-----------------------");
    delay(50);
    sending = false;
    yield();
  }
}
