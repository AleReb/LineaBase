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

////////////////////////////////////////////////////////////////////////////////////////
//parte camara se agergaron los tabs logs camGsm cam y
 en datalogger se agego una funcion para revisar la carpeta fotos si no crearla y revisar archvo logscsv si no crearlo

*/
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 2014  // Set RX buffer to 2Kb
#define SerialAT Serial1
///// tinygsm
#define stationId 0
#define firmwareVersion 0.5
#include <TinyGsmClient.h>
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);



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

const int cantidadVariables = 4;                   // Cantidad datos a leer con sensores
const int totalVariables = cantidadVariables + 3;  // Incluye Variables genéricas (batería, señal, temperatura interna)

float dataToStore[totalVariables - 1];
float dataToSend[totalVariables - 1];
const int idsSensores[totalVariables] = { 136, 137, 138, 153, 153, 154, 139 };
const int idsVariables[totalVariables] = { 22, 23, 4, 3, 6, 3, 15 };


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
#define RS485_EN 25  // DE/RE
#define BAT_PIN 33   //cambiar segun placa madre
#define SD_CS_PIN 4  // segun placa madre
// --- LIBRERIAS ---
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"
#include "RTClib.h"            // Instalar
#include <Adafruit_PCF8574.h>  // Instalar
#include <ModbusMaster.h>      // Instalar
#include <SoftwareSerial.h>
#include <Adafruit_SHT4x.h>
//////////////////////////////////////////CAMARA
//CAMARA
//int IDCAM = 2;
// --- Transfer settings foto ---------------------------------------------------
int IDCAM = 2;  //id de camara idealmente hay que unificar con el numero del la estacion
#define CHUNK_SIZE 256
int RECEIVE_TIMEOUT = 45000;     // este timeoyt va a ser nas grande cuando lo intente por segunda vez solo para preubas
unsigned long lastByteTime = 0;  // To track last received byte time
bool sendAfterReceive = false;   // para reenviar foto
bool receiving = false;          //bool cuando esta recibiendo por serial
bool SavedSDafter = false;       // para asegurarse de que se guardo en la sd
File outFile;                    //tambien relacionado con el manejo de las fotos
String filename;                 //del archivo que se recepciona foto en la sd
String logFilePath;              //string del log file
int fileSize = 0;                //del string recibido para parsear el tama;o del archivo foto
int bytesReceived = 0;           //del manejo foto
String PHOTO_PATH = "";          ///////////-------------------------- // Ensure PHOTO_PATH is always initialized para mandar la foto por gsm
bool hasRetried = false;         //del manejo de fotos
bool photosenderOK = false;      //recibi ready de la rpi
bool requestedfoto = false;      // se solicito foto control de estructura maquina de estados
bool savedSD = false;            //guardando evita las otras funciones mauqina de estado
bool sending = false;            //mandando por gsm control de estado
bool leyendoDatos = false;       // leyendo sensores control de estados
int strikes = 0;                 //reintentos de la camara


//debug
const int CAM_POWER_PIN = 14;
#define FILE_RX_PIN 34
#define FILE_TX_PIN 26
SoftwareSerial fileSerial(FILE_RX_PIN, FILE_TX_PIN);
#define CAMBAUD 31250

/////////////////////////////////////////// fin camara


// Instancias
Adafruit_PCF8574 mux;
RTC_DS3231 rtc;
SoftwareSerial distanceSensor(39, 32);
SoftwareSerial rs485Serial(RS485_RX, RS485_TX);
ModbusMaster node;
Adafruit_SHT4x sht4;  // crea instancia

// Variables globales
DateTime now;
String dataMessage;

// Botón
constexpr gpio_num_t pinBotonPantalla = GPIO_NUM_36;
volatile bool buttonInterruptVar = false;
volatile uint32_t lastIRQ = 0;  // marca de tiempo para debounce



// Flags
bool modemReady = false;
// Flags no volátiles
RTC_DATA_ATTR bool readInstance = false;
RTC_DATA_ATTR bool modemInstance = false;
RTC_DATA_ATTR bool firstBoot = true;
// camara
RTC_DATA_ATTR bool firstBootCAM        = true;
// APN
char apn[] = "gigsky-02";

void IRAM_ATTR onButton() {
}

void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(1024);
  ////////////////CAM
  fileSerial.begin(CAMBAUD);
  pinMode(CAM_POWER_PIN, OUTPUT);
  ///////////////CAM
  turnOffVRM();                                       // Apaga placa hijo en la primera instancia para no tener problemas de lecutra de la bateria
  print_wakeup_reason();                              // Causa del reinicio
  esp_sleep_enable_ext0_wakeup(pinBotonPantalla, 0);  // activo BAJO

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    modoDisplay();
  }

  if (firstBoot) {
    Serial.print("\n\n --- Estacion de Monitoreo Línea Base - Nivel de Agua --- \n\n");
    Serial.print(" ------------------ id ");
    Serial.print(stationId);
    Serial.print(" | fw ");
    Serial.print(firmwareVersion);
    Serial.print(" -------------------- \n\n");

    Serial.println("\n\r\n\r--------------------------------------------------");
    Serial.println("-> First Boot");
    turnOnVRM();      // Enciende perifericos I2C
    Wire.begin();     // Activa puerto I2C
    scanAddresses();  // Verificamos puertos I2C
    splashScreen();
    //muxCycleLeds();          // Secuencia de Leds de booteo
    checkSD();       // Verificamos tarjeta SD
    checkFile();     // Verificamos que existan los CSV
    clearCache();    // Elimina y reconstruye archivo caché
    checkRTC();      // Verificamos el funcionamiento del RTC
    checkTime();     // Vemos la hora
    readSensors();   // Leemos sensores
    saveDataToSD();  // Guardamos datos en SD
    delay(1000);
    versionDisplay();
    sendData();    // Transmitimos datos a servidor
    turnOffVRM();  // Apaga placa hijo
    firstBoot = false;
    Serial.println("\n\r-> End First Boot");
    Serial.println("--------------------------------------------------\n\r\n\r");
    esp_sleep_enable_timer_wakeup(1000000 * 1);  // Dormir por 1 segundos
    esp_deep_sleep_start();
  }
}

void loop() {
  //   Mide cada 30 minutos
  Serial.println("\n\r\n\r--------------------------------------------------");
  turnOnVRM();   // Encender VRM
  Wire.begin();  // Activa puerto I2C

  muxGreenLed();  // Led Verde
  fastCheckSD();  // Verificamos tarjeta SD
  checkFile();    // Verificamos que existan los CSV
  checkRTC();     // Verificamos el funcionamiento del RTC
  checkTime();    // Vemos la hora

  readSensors();   // Tomar lecturas
  saveDataToSD();  // Guardar datos en Data y Caché

  Serial.println("\n\r\n\r--------------------------------------------------");

  now = rtc.now();

  if ((now.hour() == 0 && now.minute() < 35) || (now.hour() == 12 && now.minute() < 35)) {
    //   Transmite cada 12 horas
    sendData();
  }
  if ( firstBootCAM || now.hour() % 3 == 0 && now.minute() < 35) {
    //   Transmite cada 3 horas
    Serial.println("FUNCION FOTO AUTO");
    /////////////////////////////////////////////////////////////////////////////
    digitalWrite(CAM_POWER_PIN, HIGH);  // Prendo la camara y espero 3 minutos
    Serial.println("vamos a entrar a un loop de  3 minutos despues de prender la camara para tomar la foto y mandarla");
    unsigned long testStart = millis();
    // Duración del test: 3 minutos = 3 * 60 000 ms
    const unsigned long TEST_DURATION_MS = 3UL * 60UL * 1000UL;

    // Ejecutar durante 5 minutos
    while (millis() - testStart < TEST_DURATION_MS) {

      checkReadyTimeout();// apago la camara automaticamente esto es de la implemetnacion vieja porque ya no es necesario siempre entramos en deep sleep
      // 1) Procesar respuestas del módem
      readModemResponses();
      // 2) Si no estamos recibiendo y hay datos en fileSerial, leer cabecera
      if (!receiving && fileSerial.available()) {
        readHeader();
        yield();  // Cede CPU para evitar bloqueo
      }
      // 3) Si estamos en estado de recepción, procesar y continuar
      if (receiving) {
        processReception(); // cuando termina de recibir la foto la manda automaticamente
      }
      // 4) Pequeña pausa yield();  // Cede CPU para evitar bloqueo
      yield();  // Cede CPU para evitar bloqueo
    }

    Serial.println("salimos de los 3 minutos");
    firstBootCAM = false; 
  }
  delay(1000);

  turnOffVRM();  // Apaga placa hijo
  Serial.println("entramos a dormir");
  esp_sleep_enable_timer_wakeup(1000000 * 60 * 30);  // Dormir por 30 minutos
  esp_deep_sleep_start();
}
