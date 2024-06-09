#include "WiFiCredentials.h"
#include <WiFiS3.h>
#include <Ethernet.h>
#include <ArduinoHA.h>

#define ARDUINOHA_DEBUG 1

#define SLOTS 4
#define SENSOR_MAX_VALUE 562
#define SENSOR_MIN_VALUE 200
#define DRY_THRESHOLD 0.5

char ssid[] = WIFI_SSID;        // your network SSID (name)
char pass[] = WIFI_PASSWORD;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;


#define BROKER_ADDR IPAddress(192,168,1,31)
byte mac[] = {0x00, 0x10, 0xFA, 0x6E, 0x38, 0x4A};

int switchPin[4] = {7,8,9,10};
int sensorPin[4] = {A0,A1,A2,A3};

class Sensor {
  private:
   int value = 0;
   int minValue, maxValue; 
   int sensorPin;

  
  public:  
    Sensor(int sensorPin, int max, int min) {
      this->sensorPin = sensorPin;
      this->value = 0;
      this->minValue = min;
      this->maxValue = max;
    }

    int getMeasure() {
      return value;
    }

    float getNormalizedMeasure() {
      return 1.0f-((max(min(getMeasure(), maxValue), minValue)-minValue)*1.0)/(float)(maxValue - minValue);
    }

    void readValue() {
      value = analogRead(sensorPin);
    }

    boolean isDry() {
      return getNormalizedMeasure() < DRY_THRESHOLD;
    }
};

class WaterPump {
  private:
    int pin;

  public:
    WaterPump(int pin) {
      this->pin = pin;
    }

    void on() {
      digitalWrite(pin, LOW); 
    }

    void off() {
      digitalWrite(pin, HIGH); 
    }
};

Sensor*  sensor[4] = { 0, 0, 0, 0 };
WaterPump* pump[4] = { 0, 0, 0, 0 };

HASensor* haSensor[4] = { 0, 0, 0, 0 };

EthernetClient *client;
HADevice *device;
HAMqtt *mqtt;

void printData() {
  Serial.println("Board Information:");
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.println();
  Serial.println("Network Information:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

}

void setup()
{
  Serial.begin(9600);    

  while (!Serial);

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");

  Serial.println("----------------------------------------");
  printData();
  Serial.println("----------------------------------------");

  client = new EthernetClient();
  device = new HADevice(mac, sizeof(mac));
  mqtt = new HAMqtt(*client, *device);

  for(int i = 0;i < SLOTS; i++) {
    sensor[i] = new Sensor(sensorPin[i], SENSOR_MAX_VALUE, SENSOR_MIN_VALUE);
    pump[i] = new WaterPump(switchPin[i]);
    pinMode(switchPin[i], OUTPUT);
    // Serial.println(());
    haSensor[i] = new HASensor((String("moisture-sensor-")+String(i)).c_str());
    haSensor[i]->setIcon("mdi:home");
    haSensor[i]->setName((String("Moisture sensor ")+String(i)).c_str());
  }    

  for(int i = 0;i < SLOTS; i++) {
    pump[i]->off();
    // Serial.println((String("Pump ") + String(i) + " off"));
  }

  delay(5000);

  Ethernet.begin(mac);

  // set device's details (optional)
  device->setName("Irrigation System");
  device->setSoftwareVersion("1.0.0");

  mqtt->begin(BROKER_ADDR);

  
}

void loop()
{  
  //Read Temperature sensor value   
  for(int i = 0;i < SLOTS; i++) {
    sensor[i]->readValue();
    Serial.println("Sensor " + String(i) + " value = " + String(sensor[i]->getMeasure()) + " norm = " + String(sensor[i]->getNormalizedMeasure()));
  }
  
  delay(1000);

  for(int i = 0;i < SLOTS; i++) {   
    if(sensor[i]->isDry()) {
      pump[i]->on();
      Serial.println(String("Pump ") + String(i) + "on");
    } else {
      pump[i]->off();
      Serial.println(String("Pump ") + String(i) + "off");
    }
    haSensor[i]->setValue(String(sensor[i]->getNormalizedMeasure()).c_str());
  } 
  
  delay(1000);

  Ethernet.maintain();
  mqtt->loop();
}  


/*References_________________________________________________
https://www.arduino.cc/en/Reference/LiquidCrystalSetCursor
http://www.instructables.com/id/Connect-A-16x2-LCD-Display-To-An-Arduino/
http://www.instructables.com/id/Soil-Moisture-Sensor/
http://www.instructables.com/id/Arduino-Temperature-Sensor-Interfacing-LM35-THE-EA/
https://www.arduino.cc/en/Reference/digitalRead
http://www.instructables.com/id/ARDUINO-TEMPERATURE-SENSOR-LM35/
http://www.electroschematics.com/6519/simple-soil-moisture-sensor-arduino-project/

*/
