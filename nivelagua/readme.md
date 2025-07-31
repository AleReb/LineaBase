# Estación Base de Monitoreo de Nivel de Agua (Placa Madre Rev. B)

Este proyecto implementa una estación de monitoreo de nivel de agua diseñada para aplicaciones de nivel de agua en lagos, ríos u otros cuerpos de agua de flujo bajo. Kombina diversos sensores, almacenamiento en tarjeta SD y transmisión de datos vía red celular (SIM7600).

## Características principales

- **Medición de distancia**: Sensor ultrasonido A01NYUB con filtrado mediano para mayor estabilidad.
- **Medición de profundidad**: Sensor Modbus RS-485, lectura de registros de 32 bits y conversión a flotante.
- **Monitoreo de batería**: Divisor de voltaje calibrado y toma de muestra en arranque para precisión.
- **Sensores ambientales**: Temperatura y humedad externa con SHT40 (I2C), más temperatura interna del RTC.
- **Registro local**: Grabación de datos en archivos CSV (`data.csv` y `cache.csv`) en tarjeta SD.
- **Transmisión celular**: Envío de lotes de datos cada 12 horas (o en primera ejecución) al servidor REST mediante módulo SIM7600.
- **Modo display**: Interfaz en pantalla OLED SH1106 para visualizar datos en tiempo real al presionar un botón.
- **Bajo consumo**: Ciclo de sueño profundo entre mediciones (30 minutos) y control de VRM (5V/3.3V) para ahorro energético.

## Esquema de datos

Cada registro CSV incluye:

1. Epoch (segundos)
2. Distancia (cm)
3. Profundidad (m)
4. Voltaje de batería (V)
5. Temperatura externa (°C)
6. Humedad externa (% RH)
7. Temperatura RTC (°C)
8. Señal celular (RSSI)

Los datos se guardan en:

- `/data.csv`: histórico completo.
- `/cache.csv`: datos pendientes de envío.

## Requisitos de hardware

- **ESP32** (o similar).
- **Sensor ultrasonido A01NYUB** (UART a 9600 bps).
- **Sensor Modbus RS-485** (dirección 1, registros 0x0016–0x0017).
- **Módulo SIM7600** con antena 4G.
- **Sensor SHT4x** (I2C).
- **Multiplexor I2C PCF8574** (para LEDs de estado).
- **RTC DS3231**.
- **Pantalla OLED SH1106** (I2C).
- **Botón de usuario** conectado a GPIO36 (ext0 wakeup).
- **Tarjeta SD** con lector (CS en GPIO4).
- Resistencias para divisor de voltaje (470 kΩ / 100 kΩ).

## Asignación de pines

| Función                 | Pin GPIO | Nota                                  |
|-------------------------|----------|---------------------------------------|
| Distancia RX/TX (UART)  | 39 / 32  | SoftwareSerial                        |
| RS-485 RX/TX/DE-RE      | 35 / 27 / 25 | ModbusMaster                          |
| SD CS                   | 4        | Cartão SD                             |
| Batería (ADC)           | 33       | ADC1_CH5 (12 bit, atenuación 11 db)   |
| Módem SIM7600 RX/TX     | 17 / 16  | Serial1                               |
| VRM control             | 13       | GPIO para habilitar alimentación      |
| I2C SDA / SCL           | 21 / 22  | Pantalla, RTC, SHT4x, PCF8574         |
| Botón display (wakeup)  | 36       | Ext0 (activo bajo)                    |

## Configuración

1. Ajustar **APN** en la variable `apn[]`.
2. Definir `stationId` y `firmwareVersion` según despliegue.
3. Verificar que `BAT_PIN` y `SD_CS_PIN` correspondan al diseño de PCB.
4. Cargar la librería `TinyGsmClient` y dependencias (Wire, SD, RTClib, Adafruit_SHT4x, ModbusMaster, U8g2, etc.).

## Instalación

1. Clonar este repositorio:
2. Abrir `EstacionNivelAgua.ino` en el IDE de Arduino o PlatformIO.
3. Seleccionar la placa ESP32 correspondiente y puerto serial.
4. Compilar y cargar.
5. Conectar todos los periféricos según el diagrama de pines.

## Uso

- **Primer arranque**: Realiza calibraciones iniciales, crea archivos CSV, registra datos y envía el primer lote.
- **Modo normal**: Cada 30 min realiza una medición y guarda en la SD. Cada 12 h envía al servidor y limpia caché.
- **Modo display**: Presionar el botón conecta la pantalla por 5 min para visualización en tiempo real.

## Servidor REST

Los datos se envían a:
```
http://api-sensores.cmasccp.cl/insertarMedicion
```
Parámetros esperados:
- `times` = epoch
- `idsSensores` = lista de IDs de sensor
- `idsVariables` = lista de IDs de variables
- `valores` = lista de valores separados por comas

## Licencia

Este proyecto es software libre y de código abierto. Úsalo y modifícalo bajo los términos de la licencia MIT.

