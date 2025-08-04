// — Declare at top of file, junto a tus otras variables globales —
const unsigned long READY_TIMEOUT_MS = 60000*2;  // 2 minute timeout al parecer es suficiente para apagar la camara
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
      size_t remaining = fileSize - bytesReceived;
      size_t toRead = min(remaining, (size_t)CHUNK_SIZE);
      uint8_t buf[CHUNK_SIZE];
      size_t n = fileSerial.readBytes(buf, toRead);
      if (n > 0) {
        outFile.write(buf, n);
        bytesReceived += n;
        lastByteTime = millis();

        Serial.printf("Progress: %u/%u bytes\n", bytesReceived, fileSize);

        // Verificar si recibí todos los bytes para enviar el ACK adecuado
        if (bytesReceived == fileSize) {
          fileSerial.println("DONE"); // Último ACK especial
        } else {
          fileSerial.println("ACK"); // ACK normal
        }
        
        yield();
      }
    }

    // Timeout => intentamos un único reintento
    if (millis() - lastByteTime > RECEIVE_TIMEOUT) {
      outFile.close();
      Serial.printf("\nTIMEOUT: %u/%u bytes\n", bytesReceived, fileSize);
      fileSerial.println("NACK_TIMEOUT");

      if (strikes >= 4) {
        Serial.println("Error de transmisión de fotos por serial, no se reintentará más");
        requestedfoto = false;
       // blinkRedLed();
      } else {
        if (!hasRetried) {
          hasRetried = true;
          delay(3000);
          strikes++;
        //  blinkBlueLed();
          Serial.println("Reintentando foto, Strikes: " + String(strikes));
          fileSerial.println("foto");  // pedimos la retransmisión
        }
      }
      receiving = false;
      return;
    }
  }

  // Archivo recibido completamente
  outFile.close();
  Serial.println("\nFile saved: " + PHOTO_PATH);
  Serial.println("-----------------------------");
  
  requestedfoto = false;
  receiving = false;
  SavedSDafter = true;
  strikes = 0;
 // sendAfterReceive = true; // solo mandar automáticamente si se recibió correctamente una foto esto ene stacion de aire simplificado para lo demas
  sendFOTO();   // llamo funcion mandar foto despues de sacarla 
}



// --- New function: checkReadyTimeout(), llamar desde loop() ---
void checkReadyTimeout() {
  if (waitingForPhotoStart && !receiving) {
    if (millis() - photoRequestTimestamp >= READY_TIMEOUT_MS) {
      // timeout exceeded: simulate successful reception with error message
      requestedfoto = false;//reinciando todos los handlers, se podria simplificar para no testear y solo recibir y mandar
      receiving = false; //reinciando todos los handlers, se podria simplificar para no testear y solo recibir y mandar
      SavedSDafter = true; //reinciando todos los handlers, se podria simplificar para no testear y solo recibir y mandar
      waitingForPhotoStart = false; //reinciando todos los handlers, se podria simplificar para no testear y solo recibir y mandar
      digitalWrite(CAM_POWER_PIN, LOW);  // apago la camara no funciono, lo intentare denuevo en 3 horas
      Serial.println("Timeout desde el ready cumplido: APAGANDO DE TODOS MODOS");
    }
  }
}
