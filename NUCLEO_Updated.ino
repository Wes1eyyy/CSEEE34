// NUCLEO-64 code (send readings to esp32 and receive set values) - updated (4/12/2023)

#define NUCLEO_ADDR 9
#include <Wire.h>

// code for pH subsystem
#define SensorPin A1            //pH meter Analog output to Arduino Analog Input 0
#define Offset -3.42           //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 500
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

// heating subsystem - declare variables
int ThermistorPin = 2;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

// declare variables
float pH_reading = 5.3;
float pH_set;
float temp_reading = 35.1;
float temp_set = 32.0;
int stirSpeed_reading = 1231;
int stirSpeed_set;

void setup() {
  Wire.begin(NUCLEO_ADDR); // join i2c bus with address #8
  pinMode(3, OUTPUT);
  Wire.onReceive(receiveEvent); // receive set data from ThingsBoard
  Wire.onRequest(requestEvent); // send subsystem data to ESP32
  Serial.begin(9600);
}

// function to convert to string so data can be transmitted to ESP32 in a single string of 18 bytes.
String convertReadings(float pH, float temp, int stirSpeed){
  String output;
  if(stirSpeed >= 1000){
    output = "P" + String(pH) + ";" + "T" + String(temp) + ";" + "S" + String(stirSpeed);return "P" + String(pH) + ";" + "T" + String(temp) + ";" + "S" + String(stirSpeed);
  }
  else if(stirSpeed < 1000) {
    output = "P" + String(pH) + ";" + "T" + String(temp) + ";" + "S" + String(stirSpeed);return "P" + String(pH) + ";" + "T" + String(temp) + ";" + "S" + "0" + String(stirSpeed);
  }
  return output;
  
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup(). 

void requestEvent(){
  String data_sent = convertReadings(pH_reading, temp_reading, stirSpeed_reading);
  Wire.write(data_sent.c_str());
}


// function to convert set input data from ThingsBoard -> ESP32 to Nucleo
void ConvertSetData(String set_inputs){
  // get indexes of P (pH), T (temp), S (stirring speed)
  int pPos = set_inputs.indexOf('P');
  int tPos = set_inputs.indexOf('T');
  int sPos = set_inputs.indexOf('S');

  String pH_strSet = set_inputs.substring(pPos+1,tPos);
  String temp_strSet = set_inputs.substring(tPos+1,sPos);
  String stirSpeed_strSet = set_inputs.substring(sPos+1);

  pH_set = pH_strSet.toFloat();
  temp_set = temp_strSet.toFloat();
  char* endPtr;  // Pointer to the character after the converted number
  stirSpeed_set = strtol(stirSpeed_strSet.c_str(), &endPtr, 10);
}

// function that executes whenever data is received from master
void receiveEvent(int byteCount) {
  Wire.requestFrom(NUCLEO_ADDR, 18);
  String input_data = "";
  while (Wire.available()) {
    char c = Wire.read();
    input_data += c;
  }
  ConvertSetData(input_data);
}

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}


void loop() {
  // loop to measure pH for pH subsystem
  static unsigned long samplingTime = millis();          
  static unsigned long printTime = millis();
  static float pHValue,voltage,adc;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      adc = avergearray(pHArray, ArrayLenth);
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    pH_reading = pHValue;
    digitalWrite(LED,digitalRead(LED)^1);
    printTime=millis();
  }
  // loop to measure temperature and activate heater for heating subsystem
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T = T - 273.15;

  // assign temperature reading so it can be read to thingsboard.
  temp_reading = T;

  Serial.print("Temperature: ");
  Serial.print(T);
  Serial.println(" C");

  // Check if the real temp "T" is outside the range SetTemp ± 0.5
  if (T > temp_set + 0.5) {
    Serial.println("Temperature too high");
    analogWrite(3,56);
  }
  else if (T < temp_set - 0.5) {
    Serial.println("Temperature too low");
    analogWrite(3,127);
  }
  Serial.print("pH set: ");
  Serial.println(pH_set);
  Serial.print("Temperature: ");
  Serial.println(temp_set);
  Serial.print("Stirring speed: ");
  Serial.println(stirSpeed_set);
  delay(500);
}
