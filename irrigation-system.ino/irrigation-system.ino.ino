int MoistureSensor1 = A0;
int switch1Pin = 7;
int val;
int state = 0;

void setup()
{
    pinMode(switch1Pin, OUTPUT);
    Serial.begin(9600);    
}

void loop()
{  
  val = analogRead(MoistureSensor1); //Read Temperature sensor value 
  Serial.println(val);
  delay(1000);
  if(val > 440) {
      digitalWrite(switch1Pin, LOW); 
      Serial.println("Pump1 on");
      delay(3000);      
  } else {
      digitalWrite(switch1Pin, HIGH); 
      Serial.println("Pump1 off");
      delay(3000);      
  }
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
