void checkAndUpdateRTC(RTC_DS3231 &rtc) {
  // Fecha y hora de compilación
  DateTime compileTime = DateTime(F(__DATE__), F(__TIME__));
  // Hora actual leída del RTC
  DateTime now = rtc.now();

  // Primer caso: ha perdido la potencia
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time to compile time");
    rtc.adjust(compileTime);
    return;
  }

  // Segundo caso: el RTC está atrasado respecto a la compilación
  if (now.unixtime() < compileTime.unixtime()) {
    Serial.print("RTC is behind compile time (RTC: ");
    Serial.print(now.timestamp());
    Serial.print(", Compile: ");
    Serial.print(compileTime.timestamp());
    Serial.println("), updating RTC.");
    rtc.adjust(compileTime);
  }
}
void checkTime() {
  DateTime now = rtc.now();
  Serial.print("Revisar hora RTC: ");
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