//  Verifica la conexión del reloj
void checkRTC(){
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
  }
}

//  Consulta y muestra la hora del RTC
void checkTime(){
    DateTime now = rtc.now();
    Serial.print("Revisar hora RTC (UTC-0): ");

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print("  ");
    Serial.print(now.hour());
    Serial.print(':');
    Serial.print(now.minute());
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print("  ");
    Serial.print(now.unixtime(), DEC);
    Serial.println();
}

void blinkGreenLed(){
  muxGreenLed();
  delay(200);
  muxOffLed();
  delay(200);
}

void muxAllLed(){
  mux.begin(0x20, &Wire);
  mux.pinMode(0, OUTPUT);
  mux.pinMode(1, OUTPUT);
  mux.pinMode(2, OUTPUT);
  mux.digitalWrite(0, LOW);
  mux.digitalWrite(1, LOW);
  mux.digitalWrite(2, LOW);
}

void muxRedLed(){
  mux.begin(0x20, &Wire);
  mux.pinMode(0, OUTPUT);
  mux.pinMode(1, OUTPUT);
  mux.pinMode(2, OUTPUT);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, LOW);
}

void muxYellowLed(){
  mux.begin(0x20, &Wire);
  mux.pinMode(0, OUTPUT);
  mux.pinMode(1, OUTPUT);
  mux.pinMode(2, OUTPUT);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, LOW);
  mux.digitalWrite(2, HIGH);
}

void muxGreenLed(){
  mux.begin(0x20, &Wire);
  mux.pinMode(0, OUTPUT);
  mux.pinMode(1, OUTPUT);
  mux.pinMode(2, OUTPUT);
  mux.digitalWrite(0, LOW);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, HIGH);
}

void muxOffLed(){
  mux.begin(0x20, &Wire);
  mux.pinMode(0, OUTPUT);
  mux.pinMode(1, OUTPUT);
  mux.pinMode(2, OUTPUT);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, HIGH);
}

void muxCycleLeds(){
  mux.begin(0x20, &Wire);
  mux.pinMode(0, OUTPUT);
  mux.pinMode(1, OUTPUT);
  mux.pinMode(2, OUTPUT);
  mux.digitalWrite(0, LOW);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, HIGH);
  delay(250);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, LOW);
  mux.digitalWrite(2, HIGH);
  delay(250);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, LOW);
  delay(250);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, HIGH);
  delay(250);
  mux.digitalWrite(0, LOW);
  mux.digitalWrite(1, LOW);
  mux.digitalWrite(2, LOW);
  delay(100);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, HIGH);
  delay(200);
  mux.digitalWrite(0, LOW);
  mux.digitalWrite(1, LOW);
  mux.digitalWrite(2, LOW);
  delay(100);
  mux.digitalWrite(0, HIGH);
  mux.digitalWrite(1, HIGH);
  mux.digitalWrite(2, HIGH);
}

void scanAddresses(){
  Wire.begin();

  byte error, address;
  int nDevices;
 
  Serial.print("Direcciones I2C: "); 
  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0){
      Serial.print("0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.print(" "); 
      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("N/A");
  Serial.println();
}

