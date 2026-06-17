bool red_pressed = false;
bool green_pressed = false;

void setup()
{
  Serial.begin(9600);

  // Buttons
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  // LEDs
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
}
void loop()
{ 
  if (digitalRead(3) == 0 && green_pressed == false) {
    green_pressed = true;
    digitalWrite(5, HIGH);
    digitalWrite(7, LOW);
    Serial.println("Green button pushed");
  }
  else if (digitalRead(3) == 1 && green_pressed == true) {
    green_pressed = false;
  }

  if (digitalRead(4) == 0 && red_pressed == false) {
    red_pressed = true;
    digitalWrite(5, LOW);
    digitalWrite(7, HIGH);
    Serial.println("Red button pushed");
  }
  else if (digitalRead(4) == 1 && red_pressed == true) {
    red_pressed = false;
  }

  delay(100);
}