void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    int num = Serial.parseInt();
    Serial.println(num);
    Serial.flush();
  }
  delay(500);
}
