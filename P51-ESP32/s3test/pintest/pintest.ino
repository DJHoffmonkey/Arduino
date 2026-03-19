void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting Physical Pin Test...");

  int pins[] = {10, 11, 12, 14, 21, 47};
  for (int i = 0; i < 6; i++) {
    Serial.printf("Testing Pin %d... ", pins[i]);
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
    delay(100);
    digitalWrite(pins[i], LOW);
    Serial.println("Done.");
  }
  Serial.println("If you see this, your S3 and its pins are physically OK!");
}

void loop() {}