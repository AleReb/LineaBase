void testSensors(){
  Serial.println("testSensors()");
  
  // Leer pH
  PH.send_read_cmd();
  delay(1000);
  PH.receive_read_cmd();
  float ph = PH.get_last_received_reading();

  // Leer EC
  EC.send_read_cmd();
  delay(2000);
  EC.receive_read_cmd();
  float ec = EC.get_last_received_reading();

  // Leer temperatura (DS18B20)
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C) {
    tempC = -999.0; // Indicar un error si no se puede leer el sensor
  }

  Serial.print("pH: ");
  Serial.print(ph);
  Serial.print("  ec: ");
  Serial.print(ec);
  Serial.print("  temp: ");
  Serial.print(tempC);
  Serial.println();
  delay(500);
}




void realizarLecturaSensores() {
  Serial.println("Realizando lectura de sensores...");
  
  // Leer pH
  PH.send_read_cmd();
  delay(1000);
  PH.receive_read_cmd();
  float ph = PH.get_last_received_reading();

  // Leer EC
  EC.send_read_cmd();
  delay(2000);
  EC.receive_read_cmd();
  float ec = EC.get_last_received_reading();

  // Leer temperatura (DS18B20)
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C) {
    tempC = -999.0; // Indicar un error si no se puede leer el sensor
  }

  // Crear objeto JSON
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["PH"] = ph;
  jsonDoc["EC"] = ec;
  jsonDoc["TEMP"] = tempC;

  // Convertir JSON a string
  String data;
  serializeJson(jsonDoc, data);

  // Enviar datos en formato JSON al maestro
  preTransmission();
  rs485.println(data);
  Serial.println("Enviando datos al maestro: " + data);
  postTransmission();
  delay(500);
}

