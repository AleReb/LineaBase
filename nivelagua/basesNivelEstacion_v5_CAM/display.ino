//LIBRERÍAS
#include <U8g2lib.h>
#include <Wire.h>

//DECLARACIONES
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define centro_width 40
#define centro_height 13
static unsigned char centro_bits[] = {
   0xf8, 0x07, 0x00, 0xc0, 0x1f, 0xfc, 0x0f, 0x00, 0xf0, 0x3f, 0xfe, 0x3f,
   0x00, 0xfc, 0x7f, 0xfe, 0xff, 0x01, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff,
   0xfc, 0xff, 0xff, 0xff, 0x3f, 0xf0, 0xff, 0xff, 0xff, 0x3f, 0xf0, 0xff,
   0xff, 0xff, 0xff, 0xfc, 0xfe, 0xff, 0x81, 0xff, 0xfc, 0xfe, 0x3f, 0x00,
   0xfc, 0x7f, 0xfc, 0x0f, 0x00, 0xf0, 0x3f, 0xf8, 0x07, 0x00, 0xe0, 0x1f,
   0xc0, 0x00, 0x00, 0x00, 0x07 };

#define costado_width 44
#define costado_height 13
static unsigned char costado_bits[] = {
   0xf0, 0x03, 0x00, 0x00, 0xfc, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0xff, 0x03,
   0xfe, 0x1f, 0x00, 0xc0, 0xff, 0x03, 0xfe, 0xff, 0x00, 0xf0, 0xff, 0x07,
   0xff, 0xff, 0x9f, 0xff, 0xff, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f,
   0xfe, 0xff, 0x07, 0xff, 0xff, 0x07, 0xfe, 0x7f, 0x00, 0xf0, 0xff, 0x07,
   0xfc, 0x1f, 0x00, 0x80, 0xff, 0x03, 0xf8, 0x07, 0x00, 0x00, 0xff, 0x01,
   0xf0, 0x03, 0x00, 0x00, 0xfc, 0x00 };

#define bolita_width 14
#define bolita_height 13
static unsigned char bolita_bits[] = {
   0xf8, 0x07, 0xfc, 0x0f, 0xfe, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x3f,
   0xff, 0x3f, 0xff, 0x1f, 0xff, 0x1f, 0xfe, 0x1f, 0xfc, 0x0f, 0xf8, 0x07,
   0xe0, 0x00 };



void splashScreen(){
  u8g2.begin();
  delay(10);
  u8g2.clearBuffer();          // Borra el buffer
  u8g2.drawXBMP(30+8, 7, costado_width, costado_height, costado_bits); 
  u8g2.drawXBMP(30+1, 27, bolita_width, bolita_height, bolita_bits); 
  u8g2.drawXBMP(30+8, 45, costado_width, costado_height, costado_bits); 
  u8g2.drawXBMP(83-30, 27, centro_width, centro_height, centro_bits); 
  u8g2.sendBuffer();           // Envía el buffer a la pantalla
}

void modoDisplay(){
   Serial.println("> modoPantalla()");
   unsigned long startDisplayTime = millis();
   buttonInterruptVar = false;
   turnOnVRM();
   Wire.begin();
   u8g2.begin();
   checkRTC();
   splashScreen();
   delay(100);
   
   while(millis() - startDisplayTime < 300000){     // Durante 5 minutos
      readSensors();
      showDataDisplay();
      delay(500);
   }
   turnOffVRM();
   Serial.println("> FIN modoPantalla()");
}

void versionDisplay() {
   char linea[32];
   int w;
  
   u8g2.clearBuffer();

   /* --------- Subtítulo --------- */
   u8g2.setFont(u8g2_font_6x12_tf);            // 6×12 px
   const char *sub = "Linea Base Publica";
   w = u8g2.getStrWidth(sub);
   u8g2.drawStr((128 - w) / 2, 12, sub);   // pegado abajo

   sub = "Nivel de Agua";
   w = u8g2.getStrWidth(sub);
   u8g2.drawStr((128 - w) / 2, 23, sub);   // pegado abajo


   /* --------- Título grande --------- */
   u8g2.setFont(u8g2_font_logisoso24_tr);      // 24 px alto, ancho variable
   snprintf(linea, sizeof(linea), "LVAG %d", stationId);
   w = u8g2.getStrWidth(linea);
   u8g2.drawStr((128 - w) / 2, 50, linea);     // centrado en X, Y≈26

   u8g2.setFont(u8g2_font_7x14_tf);      // 24 px alto, ancho variable
   snprintf(linea, sizeof(linea), "FW %.1f", firmwareVersion);
   w = u8g2.getStrWidth(linea);
   u8g2.drawStr((128 - w) / 2, 64, linea);     // centrado, Y≈54


   u8g2.sendBuffer();
   }

   void showDataDisplay() {
   char line[24];
   DateTime now = rtc.now();

   u8g2.clearBuffer();
   u8g2.setFont(u8g2_font_6x12_tf);

   // Línea 1: fecha-hora
   snprintf(line, sizeof(line), "%04u-%02u-%02u %02u:%02u:%02u",
            now.year(), now.month(), now.day(),
            now.hour() - 4, now.minute(), now.second());
   u8g2.drawStr(0, 12, line);

   // Línea divisoria (y = 17 aprox. debajo del texto)
   u8g2.drawHLine(0, 17, 128);          // x-inicio, y, largo en píxeles

   // Línea 2: distancia
   snprintf(line, sizeof(line), "Dist:  %.2f [m]", distancia / 100.0);
   u8g2.drawStr(10, 28, line);

   // Línea 3: profundidad
   snprintf(line, sizeof(line), "Prof:  %.2f [m]", profundidad);
   u8g2.drawStr(10, 44, line);

   // Línea 4: batería
   snprintf(line, sizeof(line), "Batt:  %.2f [V]", bateria); //se corrigio bug
   u8g2.drawStr(10, 60, line);

   u8g2.sendBuffer();
}