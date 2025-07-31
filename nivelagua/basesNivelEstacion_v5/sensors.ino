void readSensors() {
  Serial.println();
  Serial.println("> Read Sensors");

  // readBatteryVoltage();// depreciado por error de hardware solo se lee cuando esta todo apagado
  for (int i = 0; i < 10; i++)
    readDistanceSensor();
  readDepthSensor();
  readSHTSensor();
  readTemperatureRTC();

  Serial.print("Voltaje Batería: ");
  Serial.print(bateria);
  Serial.println(" [V]");

  Serial.print("Distancia: ");
  Serial.print(distancia / 100);
  Serial.println(" [m]");

  Serial.print("Profundidad: ");
  Serial.print(profundidad, 3);
  Serial.println(" [m]");

  Serial.print("Temperatura Ambiental: ");
  Serial.print(temperatura, 3);
  Serial.println(" [C]");

  Serial.print("Humedad Ambiental: ");
  Serial.print(humedad, 3);
  Serial.println(" [% RH]");

  Serial.print("Temperatura Interna: ");
  Serial.print(temperaturaRTC, 3);
  Serial.println(" [C]");

  Serial.println();

  dataToStore[0] = distancia/100;
  dataToStore[1] = profundidad;
  dataToStore[2] = bateria;

  dataToStore[3] = temperatura;
  dataToStore[4] = humedad;
  dataToStore[5] = temperaturaRTC;
  //dataToStore[6] Es la señal telefónica (Se anexa en sendCache())
}

void readSHTSensor() {
  if (!sht4.begin())
    Serial.println("No se encontró el SHT40");
  // Opcional: precisión y calefactor
  sht4.setPrecision(SHT4X_HIGH_PRECISION);  // LOW / MED / HIGH
  sht4.setHeater(SHT4X_NO_HEATER);          // NO_HEATER o 100 mW/200 mW con tiempos
  delay(10);
  sensors_event_t humidity, temp;
  if (sht4.getEvent(&humidity, &temp)) {  // lee datos frescos
    temperatura = temp.temperature;
    humedad = humidity.relative_humidity;
  } else {
    Serial.println("Error de lectura");
    temperatura = -1;
    humedad = -1;
  }
}

void readTemperatureRTC() {
  // rtc.begin();
  // DateTime now = rtc.now();
  temperaturaRTC = rtc.getTemperature();
}

void readBatteryVoltage() {
  int raw = analogRead(BAT_PIN);
  float v_bat = raw * 5.7f * 3.3f / 4095.0f;
  bateria = v_bat + 1;
}

// --- Parámetros del filtro ---
constexpr uint8_t N_MED = 7;  // ventana de la mediana
uint16_t bufMed[N_MED] = { 0 };
uint8_t idxMed = 0;  // índice circular
bool bufLleno = false;

// --- Función auxiliar: inserta y devuelve mediana ---
static uint16_t median7(uint16_t nuevo) {
  bufMed[idxMed++] = nuevo;
  if (idxMed == N_MED) {
    idxMed = 0;
    bufLleno = true;
  }
  if (!bufLleno)
    return nuevo;  // hasta llenar el búfer
  uint16_t tmp[N_MED];
  memcpy(tmp, bufMed, sizeof(tmp));
  std::sort(tmp, tmp + N_MED);
  return tmp[N_MED / 2];
}

// --- Lectura A01NYUB filtrada ---
void readDistanceSensor() {
  constexpr uint32_t TMO = 200;  // ms de timeout
  uint8_t data[4];
  uint32_t t0;

  distanceSensor.begin(9600);
  static bool rxHigh = (digitalWrite(32, HIGH), true);  // modo “estable” una vez
  (void)rxHigh;                                         // silencia warning

  // Espera 4 bytes con timeout
  t0 = millis();
  while (distanceSensor.available() < 4) {
    if (millis() - t0 > TMO) {
      //Serial.println("TIMEOUT");
      distancia = 0;
      return;
    }
    yield();
  }

  for (uint8_t i = 0; i < 4; i++)
    data[i] = distanceSensor.read();
  distanceSensor.flush();
  distanceSensor.end();

  // Verificación de paquete
  if (data[0] != 0xFF) {
    //Serial.println("Cabecera errónea");
    //distancia = 0;
    return;
  }
  uint8_t sum = (data[0] + data[1] + data[2]) & 0xFF;
  if (sum != data[3]) {
    //Serial.println("Checksum erróneo");
    //distancia = 0;
    return;
  }

  uint16_t raw = (data[1] << 8) | data[2];  // mm
  if (raw < 280) {
    //Serial.println("Bajo límite");
    distancia = 0;
    return;
  }

  // --- Filtro mediana ---
  uint16_t filt = median7(raw);
  distancia = filt / 10.0;  // → cm
}

/*
void readDistanceSensor(){
  unsigned char data[4] = {};
  float distance;

  distanceSensor.begin(9600);
  digitalWrite(32, HIGH);
  delay(100);
  do {
    for (int i = 0; i < 4; i++){
      data[i] = distanceSensor.read();
    }
  }
  while (distanceSensor.read() == 0xff);
  distanceSensor.flush();
  Serial.print("Distancia: ");

  if (data[0] == 0xff){
    int sum;
    sum = (data[0] + data[1] + data[2]) & 0x00FF;
    if (sum == data[3]){
      distance = (data[1] << 8) + data[2];
      if (distance > 280){
        distancia = distance / 10;
        Serial.print(distancia);
        Serial.println(" (cm)");
      }
      else{
        Serial.println("Below the lower limit");
        distancia = -1;
      }
    }
    else{
      Serial.println("Error");
      distancia = -1;
    }
  }  
  distanceSensor.end();
}
*/

void preTx() {
  digitalWrite(RS485_EN, HIGH);
}
void postTx() {
  digitalWrite(RS485_EN, LOW);
}

void readDepthSensor() {
  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);  // recepción por defecto
  rs485Serial.begin(9600);
  node.begin(1, rs485Serial);  // ID = 1
  node.preTransmission(preTx);
  node.postTransmission(postTx);

  delay(100);

  uint8_t err = node.readHoldingRegisters(0x0016, 2);
  if (err == node.ku8MBSuccess) {
    union {
      uint32_t u32;
      float f;
    } v;
    v.u32 = ((uint32_t)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);
    profundidad = v.f;
    //Serial.print("Profundidad: ");
    //Serial.println(v.f, 3);
  } else {
    Serial.printf("Error Modbus: %u\n", err);
  }

  rs485Serial.end();  // 1) Detiene la ISR y deja el pin en OUTPUT-HIGH
  //digitalWrite(RS485_TX, LOW);   // 2) Fuerza LOW un instante (vacía el C)
  //digitalWrite(RS485_RX, LOW);   // 2) Fuerza LOW un instante (vacía el C)
  pinMode(RS485_TX, INPUT);  // 3) Lo libera totalmente (alta impedancia)
  pinMode(RS485_RX, INPUT);  // 3) Lo libera totalmente (alta impedancia)
  delay(5);                  // con R=570 kΩ y C=1 µF bastan unos ms
}