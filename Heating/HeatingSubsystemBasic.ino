int ThermistorPin = 0;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
float setTemp = 30.0; // remove this line- setTemp should be retrieved from esp32

void setup() {
  Serial.begin(9600);
  pinMode(3, OUTPUT);
  Serial.print("Set Temperature is set to: ");
  Serial.print(setTemp);
  Serial.println(" C");
}

void loop() {

  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T = T - 273.15;

  Serial.print("Temperature: ");
  Serial.print(T);
  Serial.println(" C");

  // Check if the real temp "T" is outside the range SetTemp ± 0.5
  if (T > setTemp + 0.5) {
    Serial.println("Temperature too high");
    analogWrite(3,56);
  }
  else if (T < setTemp - 0.5) {
    Serial.println("Temperature too low");
    analogWrite(3,127);
  }

  delay(500);
}

