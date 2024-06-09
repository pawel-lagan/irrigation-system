#define SLOTS 1
#define SENSOR_MAX_VALUE 562
#define SENSOR_MIN_VALUE 200
#define DRY_THRESHOLD 0.5

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


void setup()
{
  Serial.begin(9600);    

  for(int i = 0;i < SLOTS; i++) {
    sensor[i] = new Sensor(sensorPin[i], SENSOR_MAX_VALUE, SENSOR_MIN_VALUE);
    pump[i] = new WaterPump(switchPin[i]);
    pinMode(switchPin[i], OUTPUT);
    // Serial.println((String("SLOT init ")+String(i)));
  }    

  for(int i = 0;i < SLOTS; i++) {
    pump[i]->off();
    // Serial.println((String("Pump ") + String(i) + " off"));
  }

  delay(5000);
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
  }

  delay(1000);  
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
