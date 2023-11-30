// NUCLEO-64 code (send readings to esp32 and receive set values)

#define NUCLEO_ADDR 9
#include <Wire.h>

// declare variables
float pH_reading = 5.4;
float pH_set;
float temp_reading = 32.4;
float temp_set;
int stirSpeed_reading = 1000;
int stirSpeed_set;

void setup() {
  Serial.begin(9600);
  Wire.begin(NUCLEO_ADDR); // join i2c bus with address #8
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}


String convertReadings(float pH, float temp, int stirSpeed){
  return "P" + String(pH) + ";" + "T" + String(temp) + ";" + "S" + String(stirSpeed);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup(). 

void requestEvent(){
  String data_sent = convertReadings(pH_reading, temp_reading, stirSpeed_reading);
  Wire.write(data_sent.c_str());
}

void receiveEvent(int bytes){
  if(Wire.available() > 0){
    String bytes = Wire.readString();
    String ID = bytes.substring(0,1);
    String data = bytes.substring(1);

    if(ID == "P"){
      pH_set = data.toFloat();
    }
    else if(ID == "T"){
      temp_set = data.toFloat();
    }
    else if(ID == "S"){
      stirSpeed_set = data.toInt();
    }
  }
}

void loop() {
  delay(100);
}
