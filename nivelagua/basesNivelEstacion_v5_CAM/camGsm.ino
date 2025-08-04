#include <ArduinoJson.h>  //para parsear las respuestas
const char *NGROK_URL = "https://hermit-harmless-wildly.ngrok-free.app/agregarImagen";
//modem helpers
bool httpSessionActive = false;
String lastAtCommand;
String httpReadData;
// --- Globals for photo-logging ---
bool photoPendingLog = false;

// — Wrapper: send AT command synchronously and parse any ERROR
//   Returns true if response was OK, false otherwise.
bool sendAtSync(const String &cmd, String &resp, unsigned long timeout = 2000) {
  // flush any stray URCs
  unsigned long t0 = millis();
  while (millis() - t0 < 100 && modem.stream.available()) {
    modem.stream.readStringUntil('\n');
  }

  lastAtCommand = cmd;
  Serial.println("AT> " + cmd);
  modem.sendAT(cmd);
  int st = modem.waitResponse(timeout, resp);
  Serial.println("AT< " + resp);

  if (st != 1 || resp.startsWith("ERROR") || resp.startsWith("+CME ERROR")) {
    Serial.println("AT command error after " + lastAtCommand + ": " + resp);
    // special case: ignore HTTPTERM failures
    if (cmd == "+HTTPTERM") {
      Serial.println(" no lo saltamos para probar volver al original si funciona el log // Saltando error de cierre");
      logError("+HTTPTERM", lastAtCommand, resp);//comentar esto si parsea este error y otros
      return true;
    }
    if (cmd == " +HTTPDATA") {
      Serial.println("Saltando error de DATA");
      return true;
    }
    if (cmd == "+HTTPINIT") {
      Serial.println("Guardando parseo desde ASynkAT");
      logError("AT_ERROR", lastAtCommand, resp);
    }
    //
    return false;
  }
  return true;
}

// — Close HTTP session if active
bool closeHttpSession() {
  if (!httpSessionActive) {
    Serial.println("HTTP session not active; Closed anyway");
    // return true;
  }
  String resp;
  if (sendAtSync("+HTTPTERM", resp, 2000)) {
    Serial.println("HTTP session closed");
  } else {
    Serial.println("Failed to terminate HTTP session");
  }
  httpSessionActive = false;
  return true;
}


// --- Process asynchronous modem events ----------------------------------
// y solo guarda en el log la línea de "Image saved at ..." cuando realmente existe.
// Reads and processes asynchronous modem events, with full logging
void readModemResponses() {
  while (modem.stream.available()) {
    String line = modem.stream.readStringUntil('\n');
    line.trim();

    // — No GPRS network event —
    if (line.startsWith("+HTTP_NONET_EVENT")) {
      Serial.println("Error: no GPRS network for HTTP (+HTTP_NONET_EVENT)");
      logError("HTTP_NONET_EVENT", "", line);
      closeHttpSession();
    }
    // — HTTP action result —
    else if (line.startsWith("+HTTPACTION:")) {
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      int status = line.substring(c1 + 1, c2).toInt();
      int length = line.substring(c2 + 1).toInt();
      Serial.printf("HTTPACTION status=%d length=%d\n", status, length);

      if (status == 200 && length > 0) {
        // Standard OK, read payload but do not log as event
        sendAtSync(String("+HTTPREAD=0,") + length, line, 5000);
        Serial.println("200 Created — reading payload for photo log");
        photoPendingLog = true;
      } else if (status == 201 && length > 0) {
        // Created — read payload and mark for photo log
        sendAtSync(String("+HTTPREAD=0,") + length, line, 5000);
        Serial.println("201 Created — reading payload");
      } else if (status == 500 && length > 0) {
        // Server error — read payload and log error
        sendAtSync(String("+HTTPREAD=0,") + length, line, 5000);
        Serial.println("500 Server Error — reading payload");
        logError("HTTPACTION_500", String(status), line);
      } else {
        // Any other status — log and close session
        Serial.printf("HTTPACTION Error: status code %d length=%d\n", status, length);
        logError("HTTPACTION_ERROR", String(status), line);
        closeHttpSession();
      }
    }
    // — HTTP read payload —
    else if (line.startsWith("+HTTPREAD:")) {
      int comma = line.indexOf(',');
      int length = (comma > 0) ? line.substring(comma + 1).toInt() : 0;
      httpReadData = "";
      unsigned long start = millis();
      while (millis() - start < 12000 && httpReadData.length() < length) {
        if (modem.stream.available()) {
          httpReadData += (char)modem.stream.read();
        }
      }
      Serial.println("HTTPREAD data: " + httpReadData);

      // If flagged for photo log, parse JSON and record event
      if (photoPendingLog) {
        // Temporary strings for parsed values
        String parsedId;
        String parsedMsg;

        // Parse JSON into a local document
        DynamicJsonDocument doc(1024);
        auto err = deserializeJson(doc, httpReadData);
        if (!err) {
          // Extract into temporaries
          parsedId = String(doc["api_response"]["id"].as<int>());
          parsedMsg = String(doc["message"].as<const char *>());

          // Print to Serial for verification
          Serial.println("Parsed ID: " + parsedId); // aqui tambien
          Serial.println("Parsed message: " + parsedMsg); // aqui funciono
          // Extract the top-level "message"
          String parsedMsg = String(doc["message"].as<const char *>());

          // Print to Serial for verification
          Serial.println("Parsed message: " + parsedMsg);//aqui se repitio

          // Log only the message (code field left empty)
          ensureLogFileExists();
          appendFile(SD, logFilePath.c_str(), parsedMsg.c_str());
          // Now that parsing succeeded, record it in the log
        } else {
          // Report parse failure, do not crash or log
          Serial.print("JSON parse error: ");
          Serial.println(err.c_str());
        }
        photoPendingLog = false;
      }
    }
    // — HTTP peer closed —
    else if (line.startsWith("+HTTP_PEER_CLOSED")) {
      Serial.println("Error: HTTP peer closed, logging and closing session");
      logError("HTTP_PEER_CLOSED", "", line);
      closeHttpSession();
    }
    // — AT-level or CME errors —
    else if (line.startsWith("ERROR") || line.startsWith("+CME ERROR")) {
      Serial.println("AT command error after " + lastAtCommand + ": " + line);
      logError("AT_ERROR", lastAtCommand, line);
      if (httpSessionActive && lastAtCommand != "+HTTPTERM") {
        closeHttpSession();
      }
    }
  }
}

// — Send image via HTTP POST (example)
void sendImageWebhook() {
  checkSD();  // Verificamos tarjeta SD
  // 1) Extract filename & sensorId from PHOTO_PATH
  String filename = PHOTO_PATH.substring(PHOTO_PATH.lastIndexOf('/') + 1);
  int us = filename.indexOf('_');
  if (us < 0) {
    Serial.println("Error: invalid filename format");
    return;
  }
  String sensorId = filename.substring(0, us);
  String fullUrl = String(NGROK_URL) + "?id_sensor=" + sensorId + "&filename=" + filename;

  // 2) Close any previous HTTP session
  closeHttpSession();

  // 3) HTTPINIT
  String resp;
  if (!sendAtSync("+HTTPINIT", resp, 5000)) {
    Serial.println("HTTPINIT failed");
    return;
  }

  httpSessionActive = true;

  // 4) Set HTTP parameters
  sendAtSync("+HTTPPARA=\"CID\",1", resp);
  sendAtSync(String("+HTTPPARA=\"URL\",\"") + fullUrl + "\"", resp);
  sendAtSync("+HTTPPARA=\"CONTENT\",\"image/jpeg\"", resp);

  // 5) Declare data length and wait for prompt
  size_t imgSize = SD.open(PHOTO_PATH, FILE_READ).size();
  if (!sendAtSync(String("+HTTPDATA=") + imgSize + ",5000", resp, 5000)) {
    Serial.println("No HTTPDATA prompt");
    // we continue anyway…
  }

  // 6) Stream image bytes
  File img = SD.open(PHOTO_PATH, FILE_READ);
  uint8_t buf[256];
  while (img.available()) {
    size_t n = img.read(buf, sizeof(buf));
    modem.stream.write(buf, n);
    delay(1);
  }
  img.close();
  Serial.println("Image data sent");

  // 7) Trigger POST
  if (!sendAtSync("+HTTPACTION=1", resp, 10000)) {
    Serial.println("HTTPACTION failed");
  }
  // 7) Finalmente, cierra la sesión HTTP
  closeHttpSession();
  // readModemResponses() se encargará del resto
}

void sendFOTO() {
  unsigned long modemInstanceStartTime = millis();
  Serial.println();
  turnOffVRM();  // Apaga placa hijo // puesto aqui para que el momdem siempre parta apagado
  delay(1000);   //esperamos 1 segundo con el sistema apagado
  Serial.println("> Send FOTO");
  turnOnVRM();   // aqui ya estamos prendidos, pero aseguramos prender
  Wire.begin();  // Activa puerto I2C
  checkSD();     // Verificamos tarjeta SD
  muxYellowLed();
  myModem.begin(115200, SERIAL_8N1, 17, 16);
  Serial.println("Esperando Modem");
  modemReady = false;
  bool modemConnected = true;

  unsigned long startWaitTime = millis();
  while (!myModem.available()) {
    // Espera que el modem entregue su estado inicial
    if (millis() - startWaitTime > 60000) {  //le dimos mas tiempo pero en general siempre funciona bien con 30 segundos
      Serial.println("El modem no contesta");
      modemConnected = false;
      break;
    }
  }

  if (modemConnected) {
    muxGreenLed();
    while (myModem.available()) {  // Recibe el estado inicial
      Serial.print((char)myModem.read());
    }
    //comandoAT("AT+COPS=0", "\r\nOK\r\n", 1000);

    for (int i = 0; i < 40; i++) {  // Verifica conexión
      if (connectGSM()) {
        modemReady = true;
        break;
      }
      delay(1000);
    }

    signalValue = signalQuality();

    if (modemReady) {
      connectGPRS();
      sendImageWebhook();
      closeGPRS();
    } else {
      Serial.println("No hay señal. no mandamos.");
      muxRedLed();
      delay(1000);
      muxOffLed();
    }

    modemInstance = false;
  }
  for (int i = 0; i < 5; i++)
    blinkGreenLed();
}
