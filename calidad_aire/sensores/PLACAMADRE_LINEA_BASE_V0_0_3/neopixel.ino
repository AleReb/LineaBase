void muxCycleLeds() {
  pinMode(rgbPowerPin, OUTPUT);
  digitalWrite(rgbPowerPin, HIGH);
  pixels.setPixelColor(0, 0, 0, 0);
  pixels.show();

  muxRedLed();
  delay(500);
  muxRedLed();
  delay(500);
  muxYellowLed();
  delay(500);
  muxGreenLed();
  delay(500);
  muxOffLed();
  delay(400);

  for (int i = 0; i < 3; i++) {
    muxWhiteLed();
    delay(150);
    muxOffLed();
    delay(150);
  }
}

void blinkGreenLed() {
  muxGreenLed();
  delay(200);
  muxOffLed();
  delay(200);
}
void blinkBlueLed() {
  muxBlueLed();
  delay(200);
  muxOffLed();
  delay(200);
}
void blinkRedLed() {
  muxRedLed();
  delay(200);
  muxOffLed();
  delay(200);
}

// Blink yellow LED (using existing muxYellowLed)
void blinkYellowLed() {
  muxYellowLed();
  delay(200);
  muxOffLed();
  delay(200);
}

// Blink orange LED
void blinkOrangeLed() {
  muxOrangeLed();
  delay(200);
  muxOffLed();
  delay(200);
}

void muxAllLed() {
  digitalWrite(rgbPowerPin, HIGH);
  pixels.setPixelColor(0, 150, 150, 150);
  pixels.show();
}

void muxRedLed() {
  digitalWrite(rgbPowerPin, HIGH);
  pixels.setPixelColor(0, 0, 255, 0);
  pixels.show();
}
void muxGreenLed() {
  digitalWrite(rgbPowerPin, HIGH);
  pixels.setPixelColor(0, 255, 0, 0);
  pixels.show();
}

void muxBlueLed() {
  digitalWrite(rgbPowerPin, HIGH);
  pixels.setPixelColor(0, 0, 0, 255);
  pixels.show();
}

void muxYellowLed() {
  digitalWrite(rgbPowerPin, HIGH);
  pixels.setPixelColor(0, 90, 255, 0);
  pixels.show();
}
// Static orange LED
void muxOrangeLed() {
  digitalWrite(rgbPowerPin, HIGH);
  // RGB: red=150, green=75, blue=0 for orange
  pixels.setPixelColor(0, 25,255, 0);
  pixels.show();
}

void muxWhiteLed() {
  digitalWrite(rgbPowerPin, HIGH);
  pixels.setPixelColor(0, 150, 150, 150);
  pixels.show();
}

void muxOffLed() {
  pixels.setPixelColor(0, 0, 0, 0);
  pixels.show();
  digitalWrite(rgbPowerPin, HIGH);
}
