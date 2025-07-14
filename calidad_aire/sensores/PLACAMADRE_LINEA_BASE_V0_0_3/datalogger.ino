// Encabezado CSV
const char *DATA_HEADER =
  "timestamp,fecha hora,"
  "BateriaEstacion,temperaturaEstacion,SEÑAL,"
  "adc2 CO raw OP1/CH3,CO rawOP2/ch4,"
  "adc2 N02 OP1/CH1 RAW,NO2 OP2/CH2,"
  "adc1 N0 OP1/CH3 RAW,NO OP2/CH4 RAW,"
  "adc1 OX OP1/CH1 RAW,OX OP2/CH2,"
  "PMS TEMP,PMS HUM,PMS 2.5,"
  "CO2_internal,CO2_RAW,CO2_CUSTOM,"
  "ESTACION TEMP,ESTACION HUM,ESTACION PRESION,"
  "VIENTO VELOCIDAD,VIENTO DIRECCION,"
  "LLUVIA ACUMULADA TOTAL,RADIACION TOTAL SOLAR,"
  "TOVC,SO2 RAW";

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
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("  Appending to file: %s -> ", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
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
  String line = "";
  line  = String(now.unixtime());      // timestamp
  line += "," + String(bufTime);       // fecha hora
  line += "," + String(batteryLevel, 2);
  line += "," + String(stationTemperature, 2);
  line += "," + signalQuality;
  for (uint8_t ch = 0; ch < 4; ch++)
    line += "," + String(adc2Raw[ch]);
  for (uint8_t ch = 0; ch < 4; ch++)
    line += "," + String(adc1Raw[ch]);
  line += "," + String(pmsTemp, 1);
  line += "," + String(pmsHum, 1);
  line += "," + String(pm25);
  line += "," + String(co2Ppm);
  line += "," + String(co2Raw, 2);
  line += "," + String(co2Custom, 2);
  line += "," + String(windTemperature, 2);
  line += "," + String(windHumidity, 1);
  line += "," + String(windPressure, 1);
  line += "," + String(windSpeed, 2);
  line += "," + String(windDirection);
  line += "," + String(windRainfall, 1);
  line += "," + String(windSolarIrradiance, 1);
  line += "," + String(tvocValue);
  line += "," + String(so2Ppm, 4);
  line += "\r\n";                     // Asegura salto de línea CR+LF

  // 3) Volcar al CSV
  appendFile(SD, fileCSV.c_str(), line.c_str());
  // 4) Mostrar la misma línea en Serial para confirmación
  Serial.print(line);
}
