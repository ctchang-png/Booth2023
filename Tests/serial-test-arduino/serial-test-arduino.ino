#define BUFFER_LENGTH 16

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.setTimeout(1);
  delay(500);
}


char buf[BUFFER_LENGTH];

void loop() {
  while (Serial.available() < BUFFER_LENGTH);
  int num_bytes = Serial.readBytes(buf, BUFFER_LENGTH);
  for (int i = 0; i < num_bytes; i++) {
    Serial.write(buf[i]);
  }
  delay(300);
}
