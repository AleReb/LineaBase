/*
  Estación Base de Monitoreo de Nivel de Agua (Placa Madre Rev. B)
  Implementación original por Sebastián Adonay
  Refactorizado y corregido por Alejandro Rebolledo

  Cambios realizados:
  - Apagado inicial de la VRM en setup() para evitar que el pin de batería quede flotando y garantizar lecturas precisas, 
  la lectura de la bateria solo se hace antes de prender el sistema.
  - BAT_PIN y SD_CS_PIN definidos como constantes globales para facilitar su ajuste según la placa madre.
  - idsSensores e idsVariables actualizados al siguiente orden de datos:
      0: Distancia
      1: Profundidad
      2: Voltaje de batería
      3: Temperatura externa
      4: Humedad externa
      5: Temperatura RTC
      6: Señal telefónica
*/
#define stationId 0
#define firmwareVersion 0.5

// --- DATOS ---
/*
Timestamp
---------
0 - Distancia
1 - Profundidad
2 - Voltaje Bateria
---------
Señal
*/

const int cantidadVariables = 4;                    // Cantidad datos a leer con sensores
const int totalVariables = cantidadVariables + 3;   // Incluye Variables genéricas (batería, señal, temperatura interna)

float dataToStore[totalVariables - 1];
float dataToSend[totalVariables - 1];
const int idsSensores[totalVariables] = {136, 137, 138, 153, 153, 154, 139}; 
const int idsVariables[totalVariables] = {22, 23, 4, 3, 6, 3, 15};


float distancia;
float profundidad;
float temperatura;
float humedad;

float bateria;
int signalValue;
float temperaturaRTC;

float display_batt;

// --- PINOUT ---
#define myModem Serial1
#define RS485_RX 35
#define RS485_TX 27
#define RS485_EN 25          // DE/RE
#define BAT_PIN 33 //cambiar segun placa madre
#define SD_CS_PIN 4  // segun placa madre
// --- LIBRERIAS ---
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"
#include "RTClib.h"             // Instalar
#include <Adafruit_PCF8574.h>   // Instalar
#include <ModbusMaster.h>       // Instalar
#include <SoftwareSerial.h>
#include <Adafruit_SHT4x.h>



// Instancias
Adafruit_PCF8574 mux;
RTC_DS3231 rtc;
SoftwareSerial distanceSensor(39, 32);
SoftwareSerial rs485Serial(RS485_RX, RS485_TX);
ModbusMaster    node;
Adafruit_SHT4x sht4;      // crea instancia

// Variables globales
DateTime now;
String dataMessage;

// Botón
constexpr gpio_num_t pinBotonPantalla = GPIO_NUM_36;
volatile bool buttonInterruptVar = false;
volatile uint32_t lastIRQ = 0;          // marca de tiempo para debounce



// Flags
bool modemReady = false;
// Flags no volátiles
RTC_DATA_ATTR bool readInstance = false;
RTC_DATA_ATTR bool modemInstance = false;
RTC_DATA_ATTR bool firstBoot = true;

// APN
char apn[] = "gigsky-02";

void IRAM_ATTR onButton() {
}

void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(1024);
  turnOffVRM();            // Apaga placa hijo en la primera instancia para no tener problemas de lecutra de la bateria
  print_wakeup_reason();                                 // Causa del reinicio
  esp_sleep_enable_ext0_wakeup(pinBotonPantalla, 0);     // activo BAJO

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    modoDisplay();
  }

  if(firstBoot){
    Serial.print("\n\n --- Estacion de Monitoreo Línea Base - Nivel de Agua --- \n\n");
    Serial.print(" ------------------ id ");
    Serial.print(stationId);
    Serial.print(" | fw ");
    Serial.print(firmwareVersion);
    Serial.print(" -------------------- \n\n");

    Serial.println("\n\r\n\r--------------------------------------------------");
    Serial.println("-> First Boot");
    turnOnVRM();             // Enciende perifericos I2C
    Wire.begin();            // Activa puerto I2C
    scanAddresses();         // Verificamos puertos I2C
    splashScreen();
    //muxCycleLeds();          // Secuencia de Leds de booteo
    checkSD();               // Verificamos tarjeta SD
    checkFile();             // Verificamos que existan los CSV
    clearCache();            // Elimina y reconstruye archivo caché
    checkRTC();              // Verificamos el funcionamiento del RTC
    checkTime();             // Vemos la hora
    readSensors();           // Leemos sensores
    saveDataToSD();          // Guardamos datos en SD
    delay(1000);
    versionDisplay();
    sendData();              // Transmitimos datos a servidor
    turnOffVRM();            // Apaga placa hijo
    firstBoot = false;
    Serial.println("\n\r-> End First Boot");
    Serial.println("--------------------------------------------------\n\r\n\r");
    esp_sleep_enable_timer_wakeup(1000000 * 1);          // Dormir por 1 segundos
    esp_deep_sleep_start();
  }
}

void loop() {
  //   Mide cada 30 minutos
  Serial.println("\n\r\n\r--------------------------------------------------");
  turnOnVRM();            // Encender VRM
  Wire.begin();           // Activa puerto I2C
  
  muxGreenLed();          // Led Verde
  fastCheckSD();          // Verificamos tarjeta SD
  checkFile();            // Verificamos que existan los CSV
  checkRTC();             // Verificamos el funcionamiento del RTC
  checkTime();            // Vemos la hora

  readSensors();          // Tomar lecturas
  saveDataToSD();         // Guardar datos en Data y Caché
  
  Serial.println("\n\r\n\r--------------------------------------------------");

  now = rtc.now();
  if ((now.hour() == 0 && now.minute() < 35) || (now.hour() == 12 && now.minute() < 35)){
    //   Transmite cada 12 horas
    sendData();
  }

  delay(1000);
  
  turnOffVRM();                                             // Apaga placa hijo
  esp_sleep_enable_timer_wakeup(1000000 * 60 * 30);          // Dormir por 30 minutos
  esp_deep_sleep_start();
}
