
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
  String line = String(buf)  + "," + String(errorType) + "," + errorCode + ",\"" + rawResponse + "\"," + signalValue + "\r\n";

  // 3) Append to SD card
  appendFile(SD, logFilePath.c_str(), line.c_str());

  // 4) Also print to Serial for live debugging
  Serial.print("Logged error: ");
  Serial.println(line);
}
