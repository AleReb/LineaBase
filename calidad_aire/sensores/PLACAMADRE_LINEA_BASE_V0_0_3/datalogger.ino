// Encabezado CSV
const char *DATA_HEADER =
  "timestamp,fecha hora,"
  "BateriaEstacion,temperaturaEstacion,SEÑAL,"
  "NO₂ OP1 RAW,NO₂ OP2 RAW,"
  "Co OP1 RAW,Co OP2 RAW,"
  "OX OP1 RAW,OX OP2 RAW,"
  "NO OP1 RAW,NO OP2 RAW,"
  "So2 OP1 RAW,So2 OP2 RAW,"
  "PMS TEMP,PMS HUM,PMS 10,PMS 2.5,"
  "CO2_internal,CO2_RAW,CO2_CUSTOM,"
  "ESTACION TEMP,ESTACION HUM,ESTACION PRESION,"
  "VIENTO VELOCIDAD,VIENTO DIRECCION,"
  "LLUVIA ACUMULADA TOTAL,RADIACION TOTAL SOLAR,"
  "TOVC,SO2 RAW,"
  "INDICE UV,INTESIDAD UV";

void ensureDataFileExists() {
  fileCSV = "/data" + String(stationId) + ".csv";

  // Crear el archivo con header si no existe
  if (!SD.exists(fileCSV)) {
    File dataFile = SD.open(fileCSV, FILE_WRITE);
    if (dataFile) {
      dataFile.println(DATA_HEADER);
      dataFile.close();
      Serial.print("Created and initialized ");
      Serial.println(fileCSV);
    } else {
      Serial.print("ERROR: Could not create ");
      Serial.println(fileCSV);
    }
  } else {
    Serial.print("File already exists: ");
    Serial.println(fileCSV);
  }
}

// Tu función de append ya dada:
void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("  Appending to file: %s -> ", path);
  if (SDOK = true) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
    blinkGreenLed();
    delay(100);
    blinkGreenLed();
    delay(100);
  } else {
    Serial.println("Append failed");
    blinkRedLed();
    delay(100);
    blinkRedLed();
    delay(100);
    blinkRedLed();
    delay(100);
  }
  file.close();
  }else{
    Serial.println("ERROR SD NOT PRESENT"); 
  }
}

// Nueva versión de writeDataToCSV usando appendFile
void writeDataToCSV() {
  // 1) Timestamp y fecha legible
  DateTime now = rtc.now();
  char bufTime[20];
  snprintf(bufTime, sizeof(bufTime),
           "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  // 2) Montar línea en String
  // Build CSV line using global variables
  String line = "";

  // Timestamp and metadata
  line = String(now.unixtime());          // timestamp
  line += "," + String(bufTime);          // date–time
  line += "," + String(batteryLevel, 2);  // battery level
  line += "," + String(stationTemperature, 2);
  line += "," + String(signalQuality);

  // ADC2 channels (NO₂ & CO)
  line += "," + String(adc2No2Op1Ch1Raw);
  line += "," + String(adc2No2Op2Ch2Raw);
  line += "," + String(adc2CoOp1Ch3Raw);
  line += "," + String(adc2CoOp2Ch4Raw);

  // ADC1 channels (OX & NO)
  line += "," + String(adc1OxOp1Ch1Raw);
  line += "," + String(adc1OxOp2Ch2Raw);
  line += "," + String(adc1NoOp1Ch3Raw);
  line += "," + String(adc1NoOp2Ch4Raw);

  // ADC3 channels (So2 & XX)
  line += "," + String(adc3So2Op1Ch1Raw);
  line += "," + String(adc3So2Op2Ch2Raw);
  //line += "," + String(adc1NoOp1Ch3Raw);
  //line += "," + String(adc1NoOp2Ch4Raw);

  // PMS sensor
  line += "," + String(pmsTemp, 1);
  line += "," + String(pmsHum, 1);
  line += "," + String(pm10);
  line += "," + String(pm25);

  // CO₂ sensor
  line += "," + String(co2Ppm);
  line += "," + String(co2Raw, 2);
  line += "," + String(co2Custom, 2);

  // Wind sensor
  line += "," + String(windTemperature, 2);
  line += "," + String(windHumidity, 1);
  line += "," + String(windPressure, 1);
  line += "," + String(windSpeed, 2);
  line += "," + String(windDirection);
  line += "," + String(windRainfall, 1);
  line += "," + String(windSolarIrradiance, 1);

  // VOC and SO₂
  line += "," + String(tvocValue);
  line += "," + String(so2Ppm, 4);

  // UV
  line += "," + String(uvIndex);
  line += "," + String(uvIntensity_mWcm2);

  line += "\r\n";  // end of record

  // 3) Volcar al CSV
  appendFile(SD, fileCSV.c_str(), line.c_str());
  // 4) Mostrar la misma línea en Serial para confirmación
  Serial.print(line);
}

// --- Call this once in setup() ---
void ensureLogFileExists() {
  logFilePath = "/logs" + String(stationId) + ".csv";
  if (!SD.exists(logFilePath)) {
    File logFile = SD.open(logFilePath, FILE_WRITE);
    if (logFile) {
      // Header: timestamp,errorType,errorCode,rawResponse,operator,technology,signal,connected
      logFile.println("timestamp,errorType,errorCode,rawResponse,operator,technology,signal,connected");
      logFile.close();
      Serial.println("Log file created: " + logFilePath);
    } else {
      Serial.println("ERROR: Could not create " + logFilePath);
    }
  } else {
    Serial.println("Log file exists: " + logFilePath);
  }
}

// --- Call this whenever you detect an error ---
void logError(const char *errorType,
              const String &errorCode,
              const String &rawResponse) {
  
  // 1) Generate human‑readable timestamp
  DateTime now = rtc.now();
  char buf[20];
  snprintf(buf, sizeof(buf),
           "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  // 2) Build CSV line with the new fields
  String line = String(buf) + "," + String(errorType) + "," + errorCode + ",\"" + rawResponse + "\"," + networkOperator + "," + networkTech + "," + signalQuality + "," + registrationStatus + "\r\n";

  // 3) Append to SD card
  appendFile(SD, logFilePath.c_str(), line.c_str());

  // 4) Also print to Serial for live debugging
  Serial.print("Logged error: ");
  Serial.println(line);
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listar directorios: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("No se pudo abrir directorio");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("No es un directorio");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}
