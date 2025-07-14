// — Declare at top of file, junto a tus otras variables globales —
const unsigned long READY_TIMEOUT_MS = 60000;    // 1 minute timeout
bool waitingForPhotoStart = false;
unsigned long photoRequestTimestamp = 0;

// --- Lee y parsea el encabezado "filename|size\n" ---
void readHeader() {
  filename = "";
  Serial.print("Receiving header: ");
  unsigned long start = millis();
  while (true) {
    if (fileSerial.available()) {
      char c = fileSerial.read();
      if (c == '\n') break;
      filename += c;
    }
    if (millis() - start > 2000) return;
  }

  if (filename.equalsIgnoreCase("ready")) {
    yield();
    Serial.println("Remote ready new: " + filename);
    photosenderOK = true;
    requestedfoto = true;
    sendAfterReceive = true;  // envío tras recibir para apagar y prender la camara
    blinkBlueLed();
    fileSerial.println("foto");
        // start timeout countdown
    waitingForPhotoStart = true;
    photoRequestTimestamp = millis();
    return;
  }

  int sep = filename.indexOf('|');
  if (sep < 0) {
    Serial.println("  Invalid header or ready: " + filename);
    return;
  }

  String nameOnly = filename.substring(0, sep);
  fileSize = filename.substring(sep + 1).toInt();
  Serial.printf("  File: %s  Size: %u bytes\n", nameOnly.c_str(), fileSize);
  // Construyo ruta única y abro SD
  PHOTO_PATH = String("/fotos/") + String(stationId) + "_" + nameOnly + ".jpg";
  outFile = SD.open(PHOTO_PATH, FILE_WRITE);
  if (!outFile) {
    Serial.println("ERROR: could not open " + PHOTO_PATH);
    return;
  }

  // Preparo variables de recepción
  bytesReceived = 0;
  hasRetried = false;  // reinicio antes de empezar a recibir
  lastByteTime = millis();
  receiving = true;
  Serial.println("  Start receiving data...");
  fileSerial.println("READY");  // <--- aviso de inicio
}

void processReception() {
  // Mientras falten bytes…
  while (bytesReceived < fileSize) {
    if (fileSerial.available()) {
      size_t toRead = min((size_t)fileSize - bytesReceived, (size_t)CHUNK_SIZE);
      uint8_t buf[CHUNK_SIZE];
      size_t n = fileSerial.read(buf, toRead);
      if (n > 0) {
        outFile.write(buf, n);
        bytesReceived += n;
        lastByteTime = millis();
        if (bytesReceived % (10240 / 2) < n) {
          Serial.printf("Progress: %d/%d bytes\n", bytesReceived, fileSize);
        }
        fileSerial.println("ACK");
        yield();
      }
    }

    // Timeout => intentamos un único reintento
    if (millis() - lastByteTime > RECEIVE_TIMEOUT) {
      outFile.close();
      Serial.printf("\nTIMEOUT: %u/%u bytes\n", bytesReceived, fileSize);
      fileSerial.println("NACK_TIMEOUT");

      if (strikes >= 4) {
        Serial.println("error de transmison de fotos por serial no se reintentara mas ");
        requestedfoto = false;
        blinkRedLed();
      } else {
        if (hasRetried == false) {
          hasRetried = true;
          delay(3000);
          strikes++;
          blinkBlueLed();
          Serial.println("Reintentando foto, Strikes: " + String(strikes));
          fileSerial.println("foto");  // pedimos la retransmisión
        }
      }
      receiving = false;
      return;
    }
  }

  // Ya recibimos todo
  outFile.close();
  Serial.println("\nFile saved: " + PHOTO_PATH);
  Serial.println("-----------------------------");
  fileSerial.println("DONE");
  requestedfoto = false;
  receiving = false;
  SavedSDafter = true;
  strikes = 0;  // NO reiniciamos hasRetried aquí; el próximo header hará reset.
}

// --- New function: checkReadyTimeout(), llamar desde loop() ---
void checkReadyTimeout() {
  if (waitingForPhotoStart && !receiving) {
    if (millis() - photoRequestTimestamp >= READY_TIMEOUT_MS) {
      // timeout exceeded: simulate successful reception with error message
      Serial.println("Timeout desde el ready cumplido: posible problema físico del cable a la cámara");
      requestedfoto = false;
      receiving = false;
      SavedSDafter = true;
      waitingForPhotoStart = false;
      digitalWrite(CAM_POWER_PIN, LOW);// apago la camara no funciono, lo intentare denuevo en 3 horas
      //strikes = 0;  // reset retry counter if lo deseas
    }
  }
}