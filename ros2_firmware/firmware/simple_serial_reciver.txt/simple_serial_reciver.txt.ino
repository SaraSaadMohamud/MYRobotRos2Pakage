#define Led_Pin 13
void setup() {
  // put your setup code here, to run once:
  pinMode(Led_Pin,OUTPUT);
  digitalWrite(Led_Pin,LOW);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    int x = Serial.readString().toInt();

    if(x == 0)
    {
      digitalWrite(Led_Pin,LOW);
    }
    else
    {
      digitalWrite(Led_Pin,HIGH);
    }
  }

}
