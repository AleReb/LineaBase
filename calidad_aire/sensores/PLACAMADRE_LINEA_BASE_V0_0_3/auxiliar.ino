//  Muestra en serial la causa de booteo
void print_wakeup_reason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON: Serial.println("Reset motivo: Power-on"); break;
    case ESP_RST_BROWNOUT: Serial.println("Reset motivo: Brown-out"); break;
    case ESP_RST_SW: Serial.println("Reset motivo: Software"); break;
    case ESP_RST_INT_WDT: Serial.println("Reset motivo: WDT interno"); break;
    case ESP_RST_TASK_WDT: Serial.println("Reset motivo: WDT de tarea"); break;
    case ESP_RST_WDT: Serial.println("Reset motivo: Watchdog"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("Reset motivo: Deep-sleep wake"); break;
    case ESP_RST_EXT: Serial.println("Reset motivo: Reset externo"); break;
    default: Serial.printf("Reset motivo: Otro (%d)\n", reason);
  }
}
// --- Process asynchronous modem events ----------------------------------
void readModemResponses2() {
  while (modem.stream.available()) {
    String line = modem.stream.readStringUntil('\n');
    line.trim();

    // Ignorar líneas vacías y ecos de comandos AT
    if (line.isEmpty() || line.startsWith("AT")) continue;

    if (line.startsWith("+HTTPACTION:")) {
      // Parsear estado y longitud
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      int status = line.substring(c1 + 1, c2).toInt();
      int length = line.substring(c2 + 1).toInt();
      Serial.printf("HTTPACTION status=%d length=%d\n", status, length);

      if ((status == 200 || status == 201) && length > 0) {
        modem.sendAT(String("+HTTPREAD=0,") + length);
      } else {
        Serial.printf("HTTPACTION Error: status code %d\n", status);
        closeHttpSession();
      }

    } else if (line.startsWith("+HTTPREAD: DATA,")) {
      // Leer la cantidad de bytes indicada
      int comma = line.indexOf(',');
      int length = line.substring(comma + 1).toInt();
      httpReadData = "";
      unsigned long t0 = millis();
      while (millis() - t0 < 12000 && httpReadData.length() < length) {
        if (modem.stream.available()) {
          httpReadData += (char)modem.stream.read();
        }
      }
      httpReadData.trim();
      Serial.println("HTTPREAD data: " + httpReadData);

      // Intentar parseo JSON
      if (httpReadData.startsWith("{") && httpReadData.endsWith("}")) {
        StaticJsonDocument<1024> doc;
        auto error = deserializeJson(doc, httpReadData);
        if (error) {
          Serial.println(F("JSON parse error"));
        } else {
          // JSON simple: mensaje y estado
          if (doc.containsKey("message")) {
            const char* msg = doc["message"];
            const char* stat = doc["status"];
            Serial.printf("Message: %s, Status: %s\n", msg, stat);
          }
          // JSON anidado: api_response
          if (doc.containsKey("api_response")) {
            auto api = doc["api_response"];
            const char* msg = api["message"];
            const char* stat = api["status"];
            Serial.printf("API Response: %s, Status: %s\n", msg, stat);
          }
        }
      }
      // Si no es JSON, comprobar si es Unix Time
      else if (isDigit(httpReadData.charAt(0))) {
        long serverTime = httpReadData.toInt();
        long localTime = rtc.now().unixtime();
        Serial.printf("Server Time: %ld, Local Time: %ld\n", serverTime, localTime);
        if (serverTime != localTime) {
          rtc.adjust(DateTime(serverTime));
          Serial.println("RTC actualizado");
        }
      } else {
        Serial.println("Unexpected HTTPREAD content");
      }

      closeHttpSession();

    } else if (line.startsWith("+CME ERROR:") || line.startsWith("ERROR")) {
      Serial.println("Modem error: " + line);
      closeHttpSession();

    } else if (line == "OK") {
      closeHttpSession();

    } else {
      Serial.println("Unhandled response: " + line);
    }
  }
}

