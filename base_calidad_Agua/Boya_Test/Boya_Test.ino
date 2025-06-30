#include <Wire.h>                 // Librería para I2C
#include <Ezo_i2c.h>              // Librería EZO I2C de Atlas Scientific
#include <SoftwareSerial.h>       // Librería para comunicación RS485
#include <ArduinoJson.h>          // Librería para manejar JSON
#include <OneWire.h>              // Librería para comunicación OneWire
#include <DallasTemperature.h>    // Librería para el sensor DS18B20
#include <ModbusMaster.h>

// Configuración del bus OneWire
OneWire oneWire(A3);
DallasTemperature sensors(&oneWire);

#define EZO_ADDRESS_PH 99    // Dirección I2C del módulo EZO para PH
#define EZO_ADDRESS_EC 100   // Dirección I2C del módulo EZO para EC

SoftwareSerial rs485(4, 2);  // RX, TX para RS485
Ezo_board PH(EZO_ADDRESS_PH, "PH");  // Crear objeto para el EZO PH
Ezo_board EC(EZO_ADDRESS_EC, "EC");  // Crear objeto para el EZO EC

const int DE_RE_PIN = 3;

void preTransmission() {
  digitalWrite(DE_RE_PIN, HIGH);
}
void postTransmission() {
  digitalWrite(DE_RE_PIN, LOW);
}

void setup() {
  Wire.begin();              // Iniciar I2C
  Serial.begin(9600);        // Iniciar Serial para monitoreo
  rs485.begin(4800);         // Iniciar RS485

  sensors.begin();           // Iniciar sensor DS18B20
  delay(1000);
  Serial.println("--- Boya ---");
  pinMode(DE_RE_PIN, OUTPUT);
  postTransmission();
  //testSensors();

}

void loop() {
  if (rs485.available()) {
    String command = rs485.readStringUntil('\n'); // Leer el comando del maestro
    command.trim(); // Eliminar espacios en blanco al inicio y al final
    Serial.println("Comando recibido del maestro: '" + command + "'");

    if (command.equalsIgnoreCase("R")) { // Comando para realizar lectura
      realizarLecturaSensores();
    } else if (command.startsWith("cal")) { // Comando de calibración
      calibrarPH(command);
    } else if (command.equalsIgnoreCase("help")) { // Comando de ayuda
      enviarComandos();
    } else {
      rs485.println("Comando no reconocido.");
      Serial.println("Comando no reconocido.");
      delay(100); // Pausa adicional para evitar que se solapen los datos
    }

    delay(100); // Pausa para el siguiente ciclo
  }
}
