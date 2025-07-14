// --- Send image via SIM7600 HTTP POST -----------------------------------
void sendImageWebhook() {
  // 1) Abrir el archivo que se acaba de guardar
  //nueva forma para testear el guardar varias fotos en archivos segun id
  // 1. Extraer el nombre del archivo de PHOTO_PATH
  String filename = "";
  int lastSlashIndex = PHOTO_PATH.lastIndexOf('/');
  if (lastSlashIndex != -1) {
    filename = PHOTO_PATH.substring(lastSlashIndex + 1);
  } else {
    filename = PHOTO_PATH;
  }

  // 2. Extraer el sensorId del 'filename'
  String sensorId = "";
  int underscoreIndex = filename.indexOf('_');
  if (underscoreIndex != -1) {
    sensorId = filename.substring(0, underscoreIndex);
  } else {
    // Manejo de error si el nombre del archivo no tiene el formato esperado
    Serial.println("Error: El nombre del archivo no contiene un ID valido (ej. ID_timestamp.jpg)");
    return;  // O maneja el error de otra manera, quizás asignando un ID por defecto
  }

  Serial.println("Filename extraído: " + filename);
  Serial.println("Sensor ID extraído: " + sensorId);

  String fullUrl = String(NGROK_URL) + "?id_sensor=" + String(sensorId) + "&filename=" + filename;

  Serial.println("OPEN FILE: " + PHOTO_PATH);
  File img = SD.open(PHOTO_PATH, FILE_READ);  //originalmente
  if (!img) {
    Serial.println("Image open failed");
    return;
  }
  size_t imgSize = img.size();
  Serial.printf("Image size: %u bytes\n", imgSize);
  // 2) Cerrar cualquier sesión HTTP previa y arrancar nuevo módulo HTTP
  modem.sendAT("+HTTPTERM");
  modem.waitResponse(2000);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000) != 1) {
    Serial.println("HTTPINIT failed");
    img.close();
    return;
  }

  // 3) Setear parámetros HTTP básicos
  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(2000);
  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + fullUrl + "\"");  //fullUrl  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + WEBHOOK_URLOLD + "\"");
  modem.waitResponse(2000);
  modem.sendAT("+HTTPPARA=\"CONTENT\",\"image/jpeg\"");
  modem.waitResponse(2000);

  // 4) Informar el tamaño que vamos a enviar y esperar prompt “>”
  modem.sendAT(String("+HTTPDATA=") + imgSize + ",10000");
  // Esperamos a “DOWNLOAD” o a “>” para empezar a subir bytes
  if (modem.waitResponse(8000, ">") != 1 && modem.waitResponse(8000, "DOWNLOAD") != 1) {
    Serial.println("No HTTPDATA prompt, continuing anyway");
  }

  // 5) Stream de bytes de la imagen
  uint8_t buf[256];
  while (img.available()) {
    size_t n = img.read(buf, sizeof(buf));
    modem.stream.write(buf, n);
    delay(1);
  }
  img.close();
  Serial.println("Image data sent");

  // 6) Esperar confirmación de OK
  modem.waitResponse(10000);

  // 7) Lanzar la acción POST
  modem.sendAT("+HTTPACTION=1");
  // La respuesta +HTTPACTION: será procesada en readModemResponses()
}
// --- Process asynchronous modem events ----------------------------------
void readModemResponses() {
  while (modem.stream.available()) {
    String line = modem.stream.readStringUntil('\n');
    line.trim();

    if (line.startsWith("+HTTPACTION:")) {
      // parse status and length
      int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1);
      int status = line.substring(c1 + 1, c2).toInt();
      int length = line.substring(c2 + 1).toInt();
      Serial.printf("HTTPACTION status=%d length=%d\n", status, length);

      if (status == 200 || status == 201 && length > 0) {
        modem.sendAT(String("+HTTPREAD=0,") + length);
      }
    } else if (line.startsWith("+HTTPREAD:")) {
      int comma = line.indexOf(',');
      int length = (comma > 0) ? line.substring(comma + 1).toInt() : 0;
      httpReadData = "";
      unsigned long t0 = millis();
      while (millis() - t0 < 12000 && httpReadData.length() < length) {
        if (modem.stream.available())
          httpReadData += (char)modem.stream.read();
      }
      Serial.println("HTTPREAD data: " + httpReadData);
      // parse JSON or timestamp as before...
    } else if (line == "OK") {
      closeHttpSession();
    }
  }
}

// --- Execute AT command with display -------------------------------------
void executeATCommand(const String &cmd, unsigned long timeout) {
  while (modem.stream.available()) modem.stream.read();
  Serial.println("AT> " + cmd);
  displayModemResponse(cmd, "");
  modem.sendAT(cmd);
  String resp;
  modem.waitResponse(timeout, resp);
  Serial.println("AT< " + resp);
  displayModemResponse(cmd, resp);
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
  executeATCommand("AT", 2000);
  executeATCommand("AT+CPIN?", 2000);
  executeATCommand("AT+CREG?", 2000);
  executeATCommand("AT+CGPADDR", 2000);
}

// --- Connect to network & GPRS -------------------------------------------
bool connectToNetwork() {
  for (int i = 0; i < 3; ++i) {
    Serial.println("Waiting for network...");
    if (modem.waitForNetwork() && modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
      Serial.println("GPRS connected");
      return true;
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

// --- Terminate HTTP session ----------------------------------------------
void closeHttpSession() {
  modem.sendAT("+HTTPTERM");
  modem.waitResponse(2000);
}

// --- Update RTC from server ----------------------------------------------
void rtcUpdate() {
  closeHttpSession();
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000) != 1) return;
  modem.sendAT("+HTTPPARA=\"URL\",\"https://southamerica-west1-fic-aysen-412113.cloudfunctions.net/unixTime\"");
  modem.waitResponse(5000);
  executeATCommand("+HTTPACTION=0", 5000);
  // readModemResponses() will handle timestamp
  closeHttpSession();
}

void UpDATA() {
  closeHttpSession();
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000) != 1) return;

  String valores = "";
  valores += String(co2Raw, 2);
  valores += "," + String(co2Ppm, 2);
 valores += "," + String(co2Custom, 2);
  valores += "," + String(pm25);
  valores += "," + String(pmsTemp, 1);
  valores += "," + String(pmsHum, 1);
  valores += "," + String(windTemperature, 2);
  valores += "," + String(windHumidity, 1);
  valores += "," + String(windSolarIrradiance, 1);
  valores += "," + String(windRainfall, 1);
  valores += "," + String(windSpeed, 2);
  valores += "," + String(windDirection);
  valores += "," + String(windPressure, 1);
  valores += "," + String(adc2Raw[0]);
  valores += "," + String(adc2Raw[1]);
  valores += "," + String(adc2Raw[2]);
  valores += "," + String(adc2Raw[3]);
  valores += "," + String(adc1Raw[0]);
  valores += "," + String(adc1Raw[1]);
  valores += "," + String(adc1Raw[2]);
  valores += "," + String(adc1Raw[3]);
  valores += "," + String(tvocValue);
  valores += "," + String(so2Ppm, 4);
  valores += "," + String(stationTemperature, 2);
  valores += "," + String(batteryLevel, 2);
  valores += "," + String(signalQuality);

  String fullUrlDATA = String(BASE_SERVER_URL) + valores;  ////1,576,96,2,3,4,5,6,7,8,9,10,66,11,12,13,14,15,16,17,18,22,23,19,20,21" //WEBHOOK_URL
 // String fullUrlDATA = String(BASE_SERVER_URL) + "1111,2222,3333,4444,5555,66666,77777,88888,99999,101010101,1111111,12121212,13131313,141414141,151515151,16161616,1171717171,18181818181,191919191,202010,21212212,222222222,232323,242424,252525,262616"; //WEBHOOK_URL
  Serial.println(fullUrlDATA);

  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + fullUrlDATA + "\"");  //fullUrl  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + WEBHOOK_URLOLD + "\"");
  modem.waitResponse(5000);
  executeATCommand("+HTTPACTION=0", 5000);
  // readModemResponses() will handle timestamp
  closeHttpSession();
}

void UpDATA3() {
  closeHttpSession();
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000) != 1) return;

  String valores = "";
  valores += String(co2Raw, 2);
  valores += "," + String(co2Ppm, 2);
 valores += "," + String(co2Custom, 2);
  valores += "," + String(pm25);
  valores += "," + String(pmsTemp, 1);
  valores += "," + String(pmsHum, 1);
  valores += "," + String(windTemperature, 2);
  valores += "," + String(windHumidity, 1);
  valores += "," + String(windSolarIrradiance, 1);
  valores += "," + String(windRainfall, 1);
  valores += "," + String(windSpeed, 2);
  valores += "," + String(windDirection);
  valores += "," + String(windPressure, 1);
  valores += "," + String(adc2Raw[0]);
  valores += "," + String(adc2Raw[1]);
  valores += "," + String(adc2Raw[2]);
  valores += "," + String(adc2Raw[3]);
  valores += "," + String(adc1Raw[0]);
  valores += "," + String(adc1Raw[1]);
  valores += "," + String(adc1Raw[2]);
  valores += "," + String(adc1Raw[3]);
  valores += "," + String(tvocValue);
  valores += "," + String(so2Ppm, 4);
  valores += "," + String(stationTemperature, 2);
  valores += "," + String(batteryLevel, 2);
  valores += "," + String(signalQuality);

  String fullUrlDATA = String(WEBHOOK_URL) + valores;  ////1,576,96,2,3,4,5,6,7,8,9,10,66,11,12,13,14,15,16,17,18,22,23,19,20,21" //WEBHOOK_URL
 // String fullUrlDATA = String(WEBHOOK_URL) + "1111,2222,3333,4444,5555,66666,77777,88888,99999,101010101,1111111,12121212,13131313,141414141,151515151,16161616,1171717171,18181818181,191919191,202010,21212212,222222222,232323,242424,252525,262616"; //WEBHOOK_URL
  Serial.println(fullUrlDATA);

  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + fullUrlDATA + "\"");  //fullUrl  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + WEBHOOK_URLOLD + "\"");
  modem.waitResponse(5000);
  executeATCommand("+HTTPACTION=0", 5000);
  // readModemResponses() will handle timestamp
  closeHttpSession();
}


void UpDATABROKEN() {
  closeHttpSession();
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000) != 1) return;
 // String fullUrlDATA = String(BASE_SERVER_URL) + valores;  ////1,576,96,2,3,4,5,6,7,8,9,10,66,11,12,13,14,15,16,17,18,22,23,19,20,21" //WEBHOOK_URL
  String fullUrlDATA = String(NGROK_URL) + "1111,2222,3333,4444,5555,66666,77777,88888,99999,101010101,1111111,12121212,13131313,141414141,151515151,16161616,1171717171,18181818181,191919191,202010,21212212,222222222,232323,242424,252525,262616"; //WEBHOOK_URL
  Serial.println(fullUrlDATA);
  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + fullUrlDATA + "\"");  //fullUrl  modem.sendAT(String("+HTTPPARA=\"URL\",\"") + WEBHOOK_URLOLD + "\"");
  modem.waitResponse(5000);
  executeATCommand("+HTTPACTION=0", 5000);
  // readModemResponses() will handle timestamp
  closeHttpSession();
}
