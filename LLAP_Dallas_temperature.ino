// LLAP temperature sensor using Dallas sensor
//
// will work with any Arduino compatible however the target boards are the
// Ciseco XinoRF and RFu-328, for LLAP over radio
//
// Uses the Ciseco LLAPSerial library
// Uses the Adafruit DHT library https://github.com/adafruit/DHT-sensor-library

#include <LLAPSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEBUG 0

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

int deviceCount = 0;
long temp;

#define DEVICEID "DG"  // this is the LLAP device ID
#define INTERVAL 10000 // millseconds
#define RADIO_PIN 8
#define RADIO_WAKE_PIN 4

void setup() {
  Serial.begin(115200);
  pinMode(RADIO_PIN, OUTPUT);   // switch on the radio
  digitalWrite(RADIO_PIN, HIGH);
  pinMode(RADIO_WAKE_PIN, OUTPUT);    // switch on the radio
  digitalWrite(RADIO_WAKE_PIN, LOW);  // ensure the radio is not sleeping
  delay(1000);        // allow the radio to startup

  sensors.begin();  // Start up the library

  deviceCount = sensors.getDeviceCount();
#if DEBUG
//  // locate devices on the bus
  Serial.print("Found ");
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
#endif    
  LLAP.init(DEVICEID);
  LLAP.sendMessage(F("STARTED"));
}

void loop() {
  // print the string when a newline arrives:
  if (LLAP.bMsgReceived) {
#if DEBUG
    Serial.print(F("msg:"));
    Serial.println(LLAP.sMessage);
#endif
    LLAP.bMsgReceived = false;  // if we do not clear the message flag then message processing will be blocked
  }

  // every N seconds
  static unsigned long lastTime = millis();
  DeviceAddress deviceAddress;
  if (millis() - lastTime >= INTERVAL) {
    lastTime = millis();
    // Send command to all the sensors for temperature conversion
    sensors.requestTemperatures();

    // Display temperature from each sensor
    for (int i = 0;  i < deviceCount;  i++)
    {
#if DEBUG
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" : ");
#endif
      if (!sensors.getAddress(deviceAddress, i)) {
        temp = 0;
#if DEBUG
        Serial.print(temp);
        Serial.print(" missing ");
#endif
      } else {
        temp = sensors.getTemp((uint8_t*) deviceAddress);
#if DEBUG
        Serial.print(temp);
        Serial.print(" raw ");
#endif
        temp = (temp * 100) / 128;
      }
#if DEBUG
      Serial.print(temp);
      Serial.println("C/100");
#endif
     LLAP.sendIntWithDP("TMP", temp, 2);
   }
   delay(1000);
  }
}
