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
/// hice una actualizacion por gsm usando la hora del modem
const long RTC_SYNC_THRESHOLD = 20;  // 20 segundos


bool getModemEpoch(time_t &epoch) {
  String resp;
  if (!sendAtSync("+CCLK?", resp, 2000)) {
    return false;
  }
  // resp example: +CCLK: "25/07/30,14:22:10+08"
  int q1 = resp.indexOf('"');
  int q2 = resp.indexOf('"', q1 + 1);
  if (q1 < 0 || q2 < 0) return false;

  String ts = resp.substring(q1 + 1, q2);
  // Split date/time
  int slash1 = ts.indexOf('/');
  int slash2 = ts.indexOf('/', slash1 + 1);
  int comma = ts.indexOf(',');
  int colon1 = ts.indexOf(':', comma);
  int colon2 = ts.indexOf(':', colon1 + 1);
  if (slash1 < 0|| slash2<0|| comma<0|| colon1<0|| colon2<0) return false;

  int yy = ts.substring(0, slash1).toInt() + 2000;
  int mo = ts.substring(slash1 + 1, slash2).toInt();
  int day = ts.substring(slash2 + 1, comma).toInt();
  int hh = ts.substring(comma + 1, colon1).toInt();
  int mm = ts.substring(colon1 + 1, colon2).toInt();
  int ss = ts.substring(colon2 + 1, colon2 + 3).toInt();

  struct tm t;
  t.tm_year = yy - 1900;
  t.tm_mon  = mo  - 1;
  t.tm_mday = day;
  t.tm_hour = hh;
  t.tm_min  = mm;
  t.tm_sec  = ss;
  t.tm_isdst = 0;

  epoch = mktime(&t);
  return true;
}

// — Synchronize onboard RTC to the modem’s clock if out of threshold
void syncRtcToModem() {
  time_t modemTime;
  if (!getModemEpoch(modemTime)) {
    Serial.println("Failed to get modem time");
    return;
  }

  DateTime rtcTime = rtc.now();
  time_t rtcEpoch = rtcTime.unixtime();

  long diff = abs((long)(modemTime - rtcEpoch));
  Serial.printf("RTC time: %lu, Modem time: %lu, diff: %ld s\n",
                rtcEpoch, modemTime, diff);

  if (diff > RTC_SYNC_THRESHOLD) {
    // Adjust RTC
    DateTime newTime = DateTime(modemTime);
    rtc.adjust(newTime);
    Serial.println("RTC synchronized to modem clock");
  } else {
    Serial.println("RTC within threshold; no sync needed");
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