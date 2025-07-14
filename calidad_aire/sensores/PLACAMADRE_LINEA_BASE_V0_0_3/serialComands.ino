// --- Handle commands from USB serial ------------------------------------
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.equalsIgnoreCase("s")) {
    sendImageWebhook();  //directo tipo foto
  } else if (cmd.equalsIgnoreCase("p")) {
    //funcion ya no se prueba porque estamos probando consumo parecido al real
    fileSerial.println("foto");
    Serial.println("Requested foto");
    requestedfoto = true;
    digitalWrite(CAM_POWER_PIN, HIGH);
    Serial.println("prendiendo rpi...");
  } else if (cmd.equalsIgnoreCase("t")) {
    rtcUpdate();  // revisar si sigue funcionando la funcion
  } else if (cmd.equalsIgnoreCase("C")) {
    Serial.println("TEST Auto request photo");
    // fileSerial.println("foto");
    digitalWrite(CAM_POWER_PIN, LOW);
    sendAfterReceive = true;  // envío tras recibir

  } else if (cmd.equalsIgnoreCase("r")) {
    UpDATA3();  //testear correctos diferentes

  } else if (cmd.equalsIgnoreCase("d")) {
    UpDATA();  //el fubcnional
  } else if (cmd.equalsIgnoreCase("e")) {
    UpDATABROKEN();  //forzar el error
  }
}