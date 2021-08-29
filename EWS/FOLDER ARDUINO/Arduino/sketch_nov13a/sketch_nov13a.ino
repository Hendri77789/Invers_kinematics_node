const int pinButton = 2;
const int pinLed = 8;
void setup () {
pinMode (pinButton, INPUT);
pinMode (pinLed, OUTPUT);
// aktifkan pull-up resistor
digitalWrite (pinButton, HIGH);
}
void loop() {
  if (digitalRead (pinButton) == LOW){
    digitalWrite (pinLed, HIGH);
    delay (100);
    digitalWrite (pinLed, LOW);
    delay (100);
  } else {
    digitalWrite (pinLed, LOW);
  }}
