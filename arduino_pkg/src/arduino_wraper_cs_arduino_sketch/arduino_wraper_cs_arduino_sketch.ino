
int chrgPin = 4;
int pumpPin = 8;
int valvPin = 7;


int counter = 0;
int sensor_value = 404;

const long interval = 1000;
unsigned long previousMillis = 0;

void setup() 
{
  Serial.begin(9600);
  
  pinMode(chrgPin, OUTPUT);
  pinMode(valvPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  
  digitalWrite(chrgPin, 0);
  digitalWrite(valvPin, 0);
  digitalWrite(pumpPin, 0);
}

void loop() 
{
  // check for serial data, if availabe then write the value to the pin
  if(Serial.available())
  {
    int x = Serial.parseInt();
    int y = Serial.parseInt();
    int z = Serial.parseInt();
    
    digitalWrite(chrgPin,x);
    digitalWrite(valvPin,y);
    digitalWrite(pumpPin,z);
  }

  // send the values over serial periodically
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
    counter += 1;
    if (counter > 2) 
    counter = 0;
    Serial.print(counter);      Serial.print(",");
    Serial.print(sensor_value); Serial.println();
  }

    

}
