//  Muestra en serial la causa de booteo
void print_wakeup_reason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON: Serial.println("Reset motivo: Power-on"); break;
    case ESP_RST_BROWNOUT: Serial.println("Reset motivo: Brown-out"); break;
    case ESP_RST_SW: Serial.println("Reset motivo: Software"); break;
    case ESP_RST_INT_WDT: Serial.println("Reset motivo: WDT interno"); break;
    case ESP_RST_TASK_WDT: Serial.println("Reset motivo: WDT de tarea"); break;
    case ESP_RST_WDT: Serial.println("Reset motivo: Watchdog"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("Reset motivo: Deep-sleep wake"); break;
    case ESP_RST_EXT: Serial.println("Reset motivo: Reset externo"); break;
    default: Serial.printf("Reset motivo: Otro (%d)\n", reason);
  }
}
