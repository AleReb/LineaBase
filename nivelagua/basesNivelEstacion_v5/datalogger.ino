void saveDataToSD(){
  DateTime now = rtc.now();
  char dataMessage[64];               // ajusta tamaño si añades campos
  int n = snprintf(dataMessage, sizeof(dataMessage), "%lu", now.unixtime());            // Primer campo (epoch)
  for (int i = 0; i < totalVariables - 1 && n < (int)sizeof(dataMessage); i++)          // Resto de datos
    n += snprintf(dataMessage + n, sizeof(dataMessage) - n, ",%.2f", dataToStore[i]);   // Cada valor con 2 decimales
  snprintf(dataMessage + n, sizeof(dataMessage) - n, "\r\n");                           // Agrega salto de línea
  
  Serial.print("> Guardando logs: ");
  Serial.println(dataMessage);

  appendFile(SD, "/data.csv", dataMessage);
  appendFile(SD, "/cache.csv", dataMessage);
}

bool leerLinea(File &f, char *buf, size_t len) {
  size_t i = 0;
  while (f.available() && i < len - 1) {
    char c = f.read();
    if (c == '\n') break;            // fin de línea
    if (c == '\r') continue;         // ignora CR
    buf[i++] = c;
  }
  buf[i] = '\0';
  return i > 0;
}

void readCache(){
  Serial.println("> Leer Caché: ");
  int filas = countFileRows(SD, "/cache.csv");
  Serial.println(filas);
  readFile(SD, "/cache.csv");
}

void checkFile(){
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  Serial.print("  Check data.csv  ");
  File file = SD.open("/data.csv");
  if(!file) {
    Serial.print("File doesn't exist  ");
    Serial.print("Creating file...  ");
    writeFile(SD, "/data.csv", "Epoch Time, Battery\r\n");
  }
  else {
    Serial.print("File already exists");  
  }
  file.close();
  Serial.println();

  Serial.print("  Check cache.csv  ");
  file = SD.open("/cache.csv");
  if(!file) {
    Serial.print("File doesn't exist  ");
    Serial.print("Creating file...  ");
    writeFile(SD, "/cache.csv", "");
  }
  else {
    Serial.print("File already exists");  
  }
  file.close();
  Serial.println();
}

// Write to the SD card
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s  ", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.print("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.print("File written  ");
  } else {
    Serial.print("Write failed  ");
  }
  Serial.println();
  file.close();
}

// Append data to the SD card
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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listar directorios: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("No se pudo abrir directorio");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("No es un directorio");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
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

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n\r", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

int countFileRows(fs::FS &fs, const char * path){
  Serial.printf("Counting Rows on file: %s  -> ", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return -1;
  }

  int rowCount = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    rowCount++;
  }

  return rowCount;
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s  ", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  }
  else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}


bool clearFile(fs::FS &fs, const char * path) {
  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("No se pudo abrir el archivo");
    return false; // Error al abrir el archivo
  }
  file.close();
  return true;
}


void clearCache(){
  Serial.print("Clear cache -> ");
  File file = SD.open("/cache.csv");
  if(!file) {
    Serial.print("File doesn't exist  ");
    Serial.print("Creating file...  ");
    createFile(SD, "/cache.csv");
  }
  else {
    deleteFile(SD, "/cache.csv");
    delay(100);
    createFile(SD, "/cache.csv");
    File file = SD.open("/cache.csv");
    if(!file) {
      Serial.println("Error creando archivo Caché");
    }
  }
  file.close();
  Serial.println();
}


void createFile(fs::FS &fs, const char * path) {
  Serial.printf("Creando archivo vacío: %s  ", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.print("> Error creando el archivo");
    muxRedLed();
    delay(1000);
    muxOffLed();
    return;
  }
  else{
    Serial.print("> Archivo creado exitosamente");
  }
  //Serial.println();
  file.close();
}


void checkSD(){
  if(!SD.begin(4)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No hay SD");
    return;
  }

  Serial.print("Tipo de SD: ");
  if(cardType == CARD_MMC){
    Serial.print("MMC");
  } else if(cardType == CARD_SD){
    Serial.print("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.print("SDHC");
  } else {
    Serial.print("UNKNOWN");
  }

  Serial.print(" - ");
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD, "/", 0);
  Serial.printf("Total space: %lluMB  ", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}


void fastCheckSD(){
  Serial.print("> Check SD: ");
  if(!SD.begin(4)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No hay SD");
    return;
  }

  Serial.print("Tipo de SD: ");
  if(cardType == CARD_MMC)         Serial.print("MMC");
  else if(cardType == CARD_SD)     Serial.print("SDSC");
  else if(cardType == CARD_SDHC)   Serial.print("SDHC");
  else                             Serial.print("UNKNOWN");
  
  Serial.print(" - ");
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB", cardSize);
  Serial.print(" - ");
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}
