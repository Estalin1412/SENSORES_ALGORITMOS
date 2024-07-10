void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  while(!Serial) delay(100);

  Serial1.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial1.available())
  {
  
    String line = Serial1.readStringUntil('\n');
    Serial.println(line);
  
  }

}
