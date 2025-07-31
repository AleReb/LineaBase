

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
      Serial.println("Saltando error de cierre");
      return true;
    }
    if (cmd == " +HTTPDATA") {
      Serial.println("Saltando error de DATA");
      return true;
    }
    if (cmd == "+HTTPINIT") {
      // special case: conntando los errores de transmison
      ModemStrikes++;
      Serial.println("Modems trikes de conexion:  " + String(ModemStrikes));
      if (ModemStrikes >= MAX_MODEM_STRIKES) {
        beginPowerCycleModem();
      }
    }

    if (SDOK = true) {
      Serial.println("Guardando parseo desde ASynkAT");
      logError("AT_ERROR", lastAtCommand, resp);
    } else {
      Serial.println("no SD init");
    }
    //
    return false;
  }
  ModemStrikes = 0;
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
// — Process asynchronous modem events: HTTPACTION, HTTPREAD, peer-closed…
void readModemResponses() {
  while (modem.stream.available()) {
    String line = modem.stream.readStringUntil('\n');
    line.trim();
    if (line.startsWith("+HTTP_NONET_EVENT")) {
      Serial.println("Error: no GPRS network for HTTP (+HTTP_NONET_EVENT)");
      logError("HTTP_NONET_EVENT", "", line);
      closeHttpSession();
    }
    // — HTTP action result
    else if (line.startsWith("+HTTPACTION:")) {
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      int status = line.substring(c1 + 1, c2).toInt();
      int length = line.substring(c2 + 1).toInt();
      Serial.printf("HTTPACTION status=%d length=%d\n", status, length);

      if ((status == 200 || status == 201) && length > 0) {
        // Read the payload
        sendAtSync(String("+HTTPREAD=0,") + length, line, 5000);
        Serial.println("OK Y leyendo dato mandandola a AsyncAT");
      } else if ((status == 500) && length > 0) {
        // Read the payload el error 500 es un error de construcion del string
        sendAtSync(String("+HTTPREAD=0,") + length, line, 5000);
        Serial.println("OK Y leyendo dato mandandola a AsyncAT con error 500");
        logError("+HTTPREAD 500", String(status), line);
      } else {
        Serial.printf("HTTPACTION Error: status code %d length=%d\n", status, length);
        Serial.println("Guardando parseo desde READ MODEM RESPONSE");
        logError("HTTPACTION", String(status), line);
        closeHttpSession();
      }
    }
    // — HTTP read payload
    else if (line.startsWith("+HTTPREAD:")) {
      int comma = line.indexOf(',');
      int length = (comma > 0) ? line.substring(comma + 1).toInt() : 0;
      httpReadData = "";
      unsigned long t0 = millis();
      while (millis() - t0 < 12000 && httpReadData.length() < length) {
        if (modem.stream.available()) {
          httpReadData += (char)modem.stream.read();
        }
      }
      Serial.println("HTTPREAD data: " + httpReadData);
    }
    // — Peer closed the connection
    else if (line.startsWith("+HTTP_PEER_CLOSED")) {
      Serial.println("Error: HTTP peer closed Saltando log");
      // logError("HTTP_PEER_CLOSED", "", line);
      closeHttpSession();
    }
    // — Other AT-level errors (CME or generic)
    else if (line.startsWith("ERROR") || line.startsWith("+CME ERROR")) {
      Serial.println("Guardando parseo desde READ MODEM RESPONSE");
      Serial.println("AT command error after " + lastAtCommand + ": " + line);
      logError("AT_ERROR", lastAtCommand, line);
      // only close if session really active and not already closing
      if (httpSessionActive && lastAtCommand != "+HTTPTERM") {
        closeHttpSession();
      }
    }
  }
}

// — Send image via HTTP POST (example)
void sendImageWebhook() {
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

// --- Display modem cmd/result on OLED -----------------------------------
void displayModemResponse(const String &cmd, const String &resp) {
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.setCursor(0, 12);
  display.print("->");
  display.print(cmd);
  display.setCursor(0, 24);
  display.print("<-");
  display.print(resp);
  display.sendBuffer();
}

// --- Basic SIM test ------------------------------------------------------
void testSIM() {
  String resp;
  sendAtSync("", resp);
  sendAtSync("+CPIN?", resp);
  sendAtSync("+CREG?", resp);
  sendAtSync("+CGPADDR", resp);
}

// --- Connect to network & GPRS -------------------------------------------
bool connectToNetwork() {
  String resp;
  for (int i = 0; i < 3; ++i) {
    Serial.println("Waiting for network...");
    if (!sendAtSync("+CREG?", resp, 5000)) {
      delay(5000);
      continue;
    }
    // Asume que "+CREG: 0,1" o "0,5" indica registro
    if (resp.indexOf("0,1") >= 0 || resp.indexOf("0,5") >= 0) {
      if (modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
        Serial.println("GPRS connected");
        return true;
      }
    }
    delay(5000);
  }
  return false;
}

// --- Fallback network info via AT ----------------------------------------
// Obtiene: Operador, Tecnología, CSQ y Registro.
void getNetworkInfoFallback(String &opOut, String &techOut, String &csqOut, String &regOut) {
  Serial.println("Ejecutando fallback para info de red...");

  // Enviar AT+COPS? para operador y tecnología
  SerialAT.println("AT+COPS?");
  delay(100);
  String copsResponse = "";
  while (SerialAT.available()) {
    copsResponse += SerialAT.readStringUntil('\n');
  }
  copsResponse.trim();
  Serial.println("Fallback COPS response: " + copsResponse);

  // Extraer operador (entre comillas)
  int firstQuote = copsResponse.indexOf("\"");
  int secondQuote = copsResponse.indexOf("\"", firstQuote + 1);
  if (firstQuote != -1 && secondQuote != -1) {
    opOut = copsResponse.substring(firstQuote + 1, secondQuote);
  } else {
    opOut = "N/A";
  }

  // Extraer tecnología (último parámetro, se espera dígito)
  int lastComma = copsResponse.lastIndexOf(",");
  String techString = "";
  if (lastComma != -1 && copsResponse.length() > lastComma + 1) {
    String techNumStr = copsResponse.substring(lastComma + 1);
    techNumStr.trim();
    int techNum = techNumStr.toInt();
    switch (techNum) {
      case 0: techString = "GSM"; break;
      case 1: techString = "GSM Compact"; break;
      case 2: techString = "UTRAN (3G)"; break;
      case 3: techString = "EDGE"; break;
      case 4: techString = "HSPA"; break;
      case 5: techString = "HSPA"; break;
      case 6: techString = "HSPA+"; break;
      case 7: techString = "LTE"; break;
      default: techString = "Unknown"; break;
    }
  } else {
    techString = "N/A";
  }
  techOut = techString;

  // Obtener CSQ mediante AT+CSQ
  SerialAT.println("AT+CSQ");
  delay(100);
  String csqResponse = "";
  while (SerialAT.available()) {
    csqResponse += SerialAT.readStringUntil('\n');
  }
  csqResponse.trim();
  // Se espera una respuesta tipo: "+CSQ: xx,yy"
  int colonIndex = csqResponse.indexOf(":");
  if (colonIndex != -1) {
    int commaIndex = csqResponse.indexOf(",", colonIndex);
    if (commaIndex != -1) {
      csqOut = csqResponse.substring(colonIndex + 1, commaIndex);
      csqOut.trim();
    } else {
      csqOut = csqResponse;
    }
  } else {
    csqOut = "N/A";
  }

  // Obtener estado de registro mediante AT+CREG?
  SerialAT.println("AT+CREG?");
  delay(100);
  String cregResponse = "";
  while (SerialAT.available()) {
    cregResponse += SerialAT.readStringUntil('\n');
  }
  cregResponse.trim();
  Serial.println("Fallback CREG response: " + cregResponse);
  if (cregResponse.indexOf("0,1") != -1 || cregResponse.indexOf("0,5") != -1) {
    regOut = "Conectado";
  } else {
    regOut = "No registrado";
  }

  // Mostrar resultados del fallback por Serial
  Serial.println("Fallback Operador: " + opOut);
  Serial.println("Fallback Tecnología: " + techOut);
  Serial.println("Fallback CSQ: " + csqOut);
  Serial.println("Fallback Registro: " + regOut);
}

// --- Update & display network info ---------------------------------------
void updateNetworkInfo() {
  getNetworkInfoFallback(networkOperator, networkTech, signalQuality, registrationStatus);
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.drawStr(0, 16, ("Op: " + networkOperator).c_str());
  display.drawStr(0, 32, ("Tec:" + networkTech).c_str());
  display.drawStr(0, 46, ("CSQ:" + signalQuality).c_str());
  display.drawStr(0, 64, ("Reg:" + registrationStatus).c_str());
  display.sendBuffer();
}


// --- Update RTC from server depreciado ----------------------------------------------

void UpDATA() {
  // 1) Cierra cualquier sesión HTTP previa
  closeHttpSession();

  String resp;
  // 2) Inicia una nueva sesión HTTP (espera hasta 20 s)
  if (!sendAtSync("+HTTPINIT", resp, 20000)) {
    Serial.println("HTTPINIT failed");
    return;
  }
  httpSessionActive = true;

  // 3) Construye la cadena de datos “valores”
  String valores = "";
  valores += String(co2Raw, 2);
  valores += "," + String(co2Ppm, 2);
  valores += "," + String(co2Custom, 2);
  valores += "," + String(pm25);
  valores += "," + String(pm10);
  valores += "," + String(pmsTemp, 1);
  valores += "," + String(pmsHum, 1);
  valores += "," + String(windTemperature, 2);
  valores += "," + String(windHumidity, 1);
  valores += "," + String(windSolarIrradiance, 1);
  valores += "," + String(windRainfall, 1);
  valores += "," + String(windSpeed, 2);
  valores += "," + String(windDirection);
  valores += "," + String(windPressure, 1);
  valores += "," + String(adc1NoOp1Ch3Raw);
  valores += "," + String(adc1NoOp2Ch4Raw);
  valores += "," + String(adc2No2Op1Ch1Raw);
  valores += "," + String(adc2No2Op2Ch2Raw);
  valores += "," + String(adc1OxOp1Ch1Raw);
  valores += "," + String(adc1OxOp2Ch2Raw);
  valores += "," + String(adc2CoOp1Ch3Raw);
  valores += "," + String(adc2CoOp2Ch4Raw);
  valores += "," + String(adc3So2Op1Ch1Raw);
  valores += "," + String(adc3So2Op2Ch2Raw);
  valores += "," + String(adc3XXOp1Ch3Raw);
  valores += "," + String(adc3XXOp2Ch4Raw);
  valores += "," + String(tvocValue);
  valores += "," + String(so2Ppm, 4);
  valores += "," + String(uvIndex);
  valores += "," + String(stationTemperature, 2);
  valores += "," + String(batteryLevel, 2);
  valores += "," + String(signalQuality);

  // 4) Monta la URL completa
  String fullUrlDATA = String(BASE_SERVER_URL) + valores;
  Serial.println(fullUrlDATA);

  // 5) Configura el parámetro URL
  if (!sendAtSync(String("+HTTPPARA=\"URL\",\"") + fullUrlDATA + "\"", resp)) {
    Serial.println("Failed to set URL parameter");
    closeHttpSession();
    return;
  }

  // (Opcional) establece CID=1 si lo necesitas
  sendAtSync("+HTTPPARA=\"CID\",1", resp);

  // 6) Lanza la petición GET y delega el parsing de +HTTPACTION a readModemResponses()
  if (!sendAtSync("+HTTPACTION=0", resp, 10000)) {
    Serial.println("HTTPACTION GET failed");
  }

  blinkBlueLed();
  // 7) Finalmente, cierra la sesión HTTP
  closeHttpSession();
}

void modemRTC() {
  String resp;
  // 2) Inicia una nueva sesión HTTP (espera hasta 20 s)
  sendAtSync("+CTZU=1", resp, 20000);
  sendAtSync("+CCLK?", resp, 20000);
}


//////////////////////////////////////////////////////////////power cicle modem
// -----------------------------------------------------------------------------
// Asynchronous modem power‑cycle state machine
// -----------------------------------------------------------------------------

enum PowerCycleState {
  PC_IDLE,
  PC_POWER_OFF,
  PC_WAIT_OFF,
  PC_POWER_ON,
  PC_WAIT_BOOT,
  PC_REINIT,
  PC_FLUSH,
  PC_DONE
};

static PowerCycleState pcState = PC_IDLE;
static unsigned long pcTimestamp = 0;

// Call this once to start the power‑cycle
void beginPowerCycleModem() {
  if (pcState != PC_IDLE) return;  // ya en marcha
  Serial.println("Begin async power‑cycle modem");
  pcState = PC_POWER_OFF;
  pcTimestamp = millis();
}

// Call this inside loop() to drive the state machine
void handlePowerCycleModem() {
  unsigned long now = millis();
  String resp;

  switch (pcState) {
    case PC_IDLE:
      // nothing to do
      break;

    case PC_POWER_OFF:
      digitalWrite(MODEM_POWER_PIN, LOW);
      Serial.println("Modem OFF");
      pcState = PC_WAIT_OFF;
      pcTimestamp = now;
      break;

    case PC_WAIT_OFF:
      if (now - pcTimestamp >= 500) {  // 0.5s elapsed
        pcState = PC_POWER_ON;
        pcTimestamp = now;
      }
      break;

    case PC_POWER_ON:
      digitalWrite(MODEM_POWER_PIN, HIGH);
      Serial.println("Modem ON");
      pcState = PC_WAIT_BOOT;
      pcTimestamp = now;
      break;

    case PC_WAIT_BOOT:
      if (now - pcTimestamp >= 2000) {  // 2s elapsed
        pcState = PC_REINIT;
      }
      break;

    case PC_REINIT:
      Serial.println("Reinitializing modem (restart + init)...");
      modem.restart();
      modem.init();
      pcState = PC_FLUSH;
      pcTimestamp = now;
      break;

    case PC_FLUSH:
      // flush any URCs for up to 200ms
      if (modem.stream.available()) {
        modem.stream.readStringUntil('\n');
        pcTimestamp = now;  // reset timeout if data still coming
      } else if (now - pcTimestamp >= 200) {
        pcState = PC_DONE;
      }
      break;

    case PC_DONE:
      Serial.println("Async power‑cycle complete");
      ModemStrikes = 0;
      pcState = PC_IDLE;
      break;
  }
}