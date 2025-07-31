# Sistema de Monitoreo Ambiental

**Autor:** Alejandro Rebolledo  
**Revisión:** 0.3  
**Fecha:** 30 de julio del 2025

## Descripción general
Este firmware implementa un sistema integrado para monitorear múltiples parámetros ambientales y climáticos utilizando una placa madre Rev. D basada en ESP32. El sistema adquiere datos desde diversos sensores y los almacena localmente en una tarjeta SD, además de enviarlos periódicamente mediante comunicación GSM/LTE al servidor remoto especificado.

## Características principales
- Adquisición de múltiples parámetros ambientales (NO₂, CO, OX, SO₂, partículas PM, CO₂, temperatura, humedad, presión, viento, lluvia, radiación solar, TVOC, UV).
- Almacenamiento en tarjeta SD en formato CSV.
- Envío periódico de datos a servidor remoto usando HTTP y módem SIM7600.
- Captura y transmisión periódica de imágenes desde una cámara externa mediante UART.
- Uso eficiente del watchdog (WDT) para estabilidad operativa.

## Variables medidas y almacenadas en SD
- Timestamp (epoch y fecha/hora humana)
- Batería de estación
- Temperatura de estación
- Calidad de señal
- Valores brutos sensores electroquímicos (NO₂, CO, OX, NO, SO₂)
- Valores ambientales del sensor PMS5003 (PM 10, PM 2.5, temperatura y humedad)
- Valores del sensor CO₂ (interno, raw, custom)
- Parámetros climáticos (temperatura, humedad, presión atmosférica)
- Datos meteorológicos (velocidad y dirección del viento, lluvia acumulada, radiación solar)
- TVOC
- Índice e intensidad UV

## Hardware requerido
- ESP32
- Módulo GSM/LTE SIM7600
- ADS1115 (ADC externo)
- MH-Z19 (sensor CO₂)
- PMS5003 (sensor PM)
- Sensores Modbus (velocidad y dirección del viento, UV)
- Sensores I²C (TVOC, SO₂)
- Tarjeta SD para almacenamiento local
- Cámara UART externa para imágenes

## Librerías requeridas
- TinyGSM
- ArduinoJson
- Adafruit ADS1115
- U8g2
- RTC_DS3231
- ModbusMaster
- MHZ19

## Instalación
1. Clonar este repositorio
```sh
git clone https://github.com/AleReb/LineaBase.git
```
2. Instalar dependencias necesarias desde el gestor de librerías de Arduino IDE.
3. Configurar los parámetros específicos (APN, URLs, intervalos de medición y envío) en el archivo principal del firmware.
4. Cargar el firmware en el ESP32 mediante Arduino IDE o plataforma similar.

## Uso
El sistema opera de manera autónoma una vez encendido. Asegúrese de verificar periódicamente:
- Estado de conexión celular.
- Integridad de los datos almacenados en la tarjeta SD.
- Nivel de batería y calidad de señal.

## Soporte y contacto
Para cualquier consulta técnica o reporte de fallos, contactar a Alejandro Rebolledo mediante los canales establecidos en este repositorio.
