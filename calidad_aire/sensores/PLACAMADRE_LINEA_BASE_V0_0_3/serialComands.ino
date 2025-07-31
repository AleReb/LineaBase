// --- Handle commands from USB serial ------------------------------------
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.equalsIgnoreCase("s")) {
    Serial.println("s: mandar foto (sendImageWebhook)");
    sendImageWebhook();
  } else if (cmd.equalsIgnoreCase("t")) {
    Serial.println("t: sincronizar RTC con fecha del módem (syncRtcToModem)");
    syncRtcToModem();
  } else if (cmd.equalsIgnoreCase("c")) {
    Serial.println("c: encender cámara y solicitar foto (auto request photo)");
    digitalWrite(CAM_POWER_PIN, HIGH);
    fileSerial.println("foto");
  } else if (cmd.equalsIgnoreCase("d")) {
    Serial.println("d: enviar datos de sensores (UpDATA)");
    UpDATA();
  } else if (cmd.equalsIgnoreCase("e")) {
    Serial.println("e: reiniciar módem (power‑cycle)");
    beginPowerCycleModem();
  } else if (cmd.equalsIgnoreCase("f")) {
    // Serial.println("f: prueba datos rota (UpDATAbroken)");
    // UpDATAbroken();
    Serial.println("f: prueba datos rota strikes reinicio de modem");
    ModemStrikes = 11;
  } else if (cmd.equalsIgnoreCase("g")) {
    Serial.println("g: test SIM y tecnología, actualizar info de red");
    testSIM();
    updateNetworkInfo();
  } else if (cmd.equalsIgnoreCase("h")) {
    Serial.println("h: mostrar esta ayuda");
    Serial.println("  s: mandar foto (HTTP POST)");
    Serial.println("  t: sincronizar RTC con módem");
    Serial.println("  C: encender cámara y pedir foto");
    Serial.println("  d: enviar datos de sensores al servidor");
    Serial.println("  e: reiniciar módem (power‑cycle async)");
    Serial.println("  f: prueba de envío de datos fallido o error de modem");
    Serial.println("  g: test SIM, registro y tecnología");
    Serial.println("  h: mostrar lista de comandos");
  }
}
