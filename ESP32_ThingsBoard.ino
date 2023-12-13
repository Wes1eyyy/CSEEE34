#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#include <Wire.h>
#endif

// define ESP32 pins
#define I2C_SDA 21
#define I2C_SCL 22

//define address of Nucleo
#define NUCLEO_ADDR 0x40

#ifndef LED_BUILTIN
#define LED_BUILTIN 99
#endif

#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <ArduinoJson.h>

constexpr char WIFI_SSID[] = "ESP32_ThingsBoard_Test";
constexpr char WIFI_PASSWORD[] = "CSEEE034";

float InputTemperature = 28;
float InputPHValue = 5.1;
int InputStirringSpeed = 1200;

// initialise readings from nucleo
float pH_reading, temp_reading;
int stirSpeed_reading;

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
constexpr char TOKEN[] = "its4rs7rhslqsp4no486";

// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port.
constexpr uint16_t THINGSBOARD_PORT = 1883U;

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

// Baud rate for the debugging serial connection.
// If the Serial output is mangled, ensure to change the monitor speed accordingly to this variable
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200;


// Initialize underlying client, used to establish a connection
WiFiClient wifiClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(wifiClient);
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// Attribute names for attribute request and attribute updates functionality

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

// handle led state and mode changes
volatile bool attributesChanged = false;

// LED modes: 0 - continious state, 1 - blinking
volatile int ledMode = 0;

// Current led state
volatile bool ledState = false;

// Settings for interval in blinking mode
constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

// For telemetry
constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;

unsigned long previousAttributeRequest = 0;
const unsigned long attributeRequestInterval = 2000;

// Statuses for requesting of attributes
bool requestedClient = false;
bool requestedShared = false;

// List of shared attributes for subscribing to their updates
constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

// List of client attributes for requesting them (Using to initialize device states)
constexpr std::array<const char *, 1U> CLIENT_ATTRIBUTES_LIST = {
  LED_MODE_ATTR
};

/// @brief Initalizes WiFi connection,
// will endlessly delay until a connection has been successfully established
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been succesfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}


/// @brief Processes function for RPC call "setLedMode"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
RPC_Response processSetLedMode(const RPC_Data &data) {
  Serial.println("Received the set led state RPC method");

  // Process data
  int new_mode = data;

  Serial.print("Mode to change: ");
  Serial.println(new_mode);

  if (new_mode != 0 && new_mode != 1) {
    return RPC_Response("error", "Unknown mode!");
  }

  ledMode = new_mode;

  attributesChanged = true;

  // Returning current mode
  return RPC_Response("newMode", (int)ledMode);
}


// Optional, keep subscribed shared attributes empty instead,
// and the callback will be called for every shared attribute changed on the device,
// instead of only the one that were entered instead
const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "setLedMode", processSetLedMode }
};



String convertInputData(float pH_input, float temp_input, int stirSpeed_input){
  String output;
  if(stirSpeed_input >= 1000){
    output = "P" + String(pH_input) + ";" + "T" + String(temp_input) + ";" + "S" + String(stirSpeed_input);
  }
  else if(stirSpeed_input < 1000){
    output = "P" + String(pH_input) + ";" + "T" + String(temp_input) + ";" + "S" + "0" + String(stirSpeed_input);
  }
  return output;
}

void sendInputData(String set_input){
  Wire.beginTransmission(NUCLEO_ADDR);
  Wire.write(set_input.c_str());
  Wire.endTransmission(NUCLEO_ADDR);
}

/// @brief Update callback that will be called as soon as one of the provided shared attributes changes value,
/// if none are provided we subscribe to any shared attribute change instead
/// @param data Data containing the shared attributes that were changed and their current value
void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), "InputTemperature") == 0) {
      float input_temp = it->value().as<float>();
      if(input_temp < 25 || input_temp > 35){
        Serial.print("InputTemperature: Invalid Input Error");
        continue;
      }
      InputTemperature = input_temp;
      Serial.print("InputTemperature: ");
      Serial.println(InputTemperature);
    }
    if (strcmp(it->key().c_str(), "InputPHValue") == 0) {
      float input_pH = it->value().as<float>();
      if(input_pH < 3 || input_pH > 7){
        Serial.print("InputPHValue: Invalid Input Error");
        continue;
      }
      InputPHValue = input_pH;
      Serial.print("InputPHValue: ");
      Serial.println(InputPHValue);
    }
    if (strcmp(it->key().c_str(), "InputStirringSpeed") == 0) {
      int input_stir_speed = it->value().as<int>();
      if(input_stir_speed < 500 || input_stir_speed > 1500){
        Serial.print("InputStirringSpeed: Invalid Input Error");
        continue;
      }
      InputStirringSpeed = input_stir_speed;
      Serial.print("InputStirringSpeed: ");
      Serial.println(InputStirringSpeed);
    }
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      if (LED_BUILTIN != 99) {
        digitalWrite(LED_BUILTIN, ledState);
      }
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  String data_to_nucleo = convertInputData(InputPHValue, InputTemperature, InputStirringSpeed);
  sendInputData(data_to_nucleo);
  attributesChanged = true;
}

void processClientAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), LED_MODE_ATTR) == 0) {
      const uint16_t new_mode = it->value().as<uint16_t>();
      ledMode = new_mode;
    }
  }
}
const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_client_request_callback(&processClientAttributes, CLIENT_ATTRIBUTES_LIST.cbegin(), CLIENT_ATTRIBUTES_LIST.cend());

void setup() {
  // Initalize serial connection for debugging
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(SERIAL_DEBUG_BAUD);
  if (LED_BUILTIN != 99) {
    pinMode(LED_BUILTIN, OUTPUT);
  }
  delay(1000);
  InitWiFi();
}

String readNucleoData(){
  Wire.requestFrom(NUCLEO_ADDR, 18);
  String nucleo_data = "";
  while(Wire.available()){
    char c = Wire.read();
    nucleo_data += c;
  }
  return nucleo_data;
}

void ConvertNucleoData(String nucleo_data){
  int pPos = nucleo_data.indexOf('P');
  int tPos = nucleo_data.indexOf('T');
  int sPos = nucleo_data.indexOf('S');

  String pH_strReading = nucleo_data.substring(pPos+1, tPos);
  String temp_strReading = nucleo_data.substring(tPos+1,sPos);
  String stirSpeed_strReading = nucleo_data.substring(sPos+1);

  pH_reading = pH_strReading.toFloat();
  temp_reading = temp_strReading.toFloat();
  char* endPtr;
  stirSpeed_reading = strtol(stirSpeed_strReading.c_str(), &endPtr, 10);
}

void loop() {
  delay(10);

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
    // Sending a MAC address as an attribute
    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());


    Serial.println("Subscribing for RPC...");
    // Perform a subscription. All consequent data processing will happen in
    // processSetLedState() and processSetLedMode() functions,
    // as denoted by callbacks array.
    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
      Serial.println("Failed to subscribe for shared attribute updates");
      return;
    }

    Serial.println("Subscribe done");

    // Request current states of shared attributes
  //   if (!requestedShared) {
  // #if THINGSBOARD_ENABLE_PROGMEM
  //     Serial.println(F("Requesting shared attributes..."));
  // #else
  //     Serial.println("Requesting shared attributes...");
  // #endif
  //     // Shared attributes we want to request from the server
  //     constexpr std::array<const char*, 3U> REQUESTED_SHARED_ATTRIBUTES = {"InputTemperature", "InputPHValue", "InputStirringSpeed"};
  //     const Attribute_Request_Callback sharedCallback(&processSharedAttributes, REQUESTED_SHARED_ATTRIBUTES.cbegin(), REQUESTED_SHARED_ATTRIBUTES.cend());
  //     requestedShared = tb.Shared_Attributes_Request(sharedCallback);
  //     if (!requestedShared) {
  // #if THINGSBOARD_ENABLE_PROGMEM
  //       Serial.println(F("Failed to request shared attributes"));
  // #else
  //       Serial.println("Failed to request shared attributes");
  // #endif
  //     }
  //}

  //   // Request current states of client attributes
  //   if (!tb.Client_Attributes_Request(attribute_client_request_callback)) { 
  //     Serial.println("Failed to request for client attributes");
  //     return;
  //   }
  }

  if (millis() - previousAttributeRequest > attributeRequestInterval) {
    previousAttributeRequest = millis();
    Serial.println("Re-requesting shared attributes...");

    // Shared attributes we want to request from the server
    constexpr std::array<const char*, 3U> REQUESTED_SHARED_ATTRIBUTES = {"InputTemperature", "InputPHValue", "InputStirringSpeed"};
    const Attribute_Request_Callback sharedCallback(&processSharedAttributes, REQUESTED_SHARED_ATTRIBUTES.cbegin(), REQUESTED_SHARED_ATTRIBUTES.cend());
    requestedShared = tb.Shared_Attributes_Request(sharedCallback);

    if (!requestedShared) {
      Serial.println("Failed to re-request shared attributes");
    }

    // Serial.print("InputTemperature: ");
    // Serial.println(InputTemperature);
    // Serial.print("InputPHValue: ");
    // Serial.println(InputPHValue);
    // Serial.print("InputStirringSpeed: ");
    // Serial.println(InputStirringSpeed);
  }


  if (attributesChanged) {
    attributesChanged = false;
    if (ledMode == 0) {
      previousStateChange = millis();
    }
    tb.sendTelemetryData(LED_MODE_ATTR, ledMode);
    tb.sendTelemetryData(LED_STATE_ATTR, ledState);
    tb.sendAttributeData(LED_MODE_ATTR, ledMode);
    tb.sendAttributeData(LED_STATE_ATTR, ledState);
  }

  if (ledMode == 1 && millis() - previousStateChange > blinkingInterval) {
    previousStateChange = millis();
    ledState = !ledState;
    tb.sendTelemetryData(LED_STATE_ATTR, ledState);
    tb.sendAttributeData(LED_STATE_ATTR, ledState);
    if (LED_BUILTIN == 99) {
      Serial.print("LED state changed to: ");
      Serial.println(ledState);
    } else {
      digitalWrite(LED_BUILTIN, ledState);
    }
  }

  // Sending telemetry every telemetrySendInterval time
  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();
    String data_from_nucleo = readNucleoData();
    ConvertNucleoData(data_from_nucleo);
    tb.sendTelemetryData("temperature", temp_reading);
    tb.sendTelemetryData("PH value", pH_reading);
    tb.sendTelemetryData("Stirring speed", stirSpeed_reading);
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
  }

  tb.loop();
}
