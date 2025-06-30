void calibrarPH(String comando) {
  Serial.println("Enviando comando de calibración: " + comando);
  PH.send_cmd(comando.c_str());
  delay(1000);

  char response[32];
  Ezo_board::errors resultado = PH.receive_cmd(response, sizeof(response));

  if (resultado == Ezo_board::SUCCESS) {
    Serial.println("Calibración exitosa, respuesta: " + String(response));
    rs485.println("Calibración exitosa");
  } else {
    Serial.println("Error en la calibración, código de error: " + String(resultado));
    rs485.println("Error en la calibración, código de error: " + String(resultado));
  }

  delay(100);
}
