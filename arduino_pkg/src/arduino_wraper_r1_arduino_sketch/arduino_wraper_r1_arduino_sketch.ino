
int chrgPin = 4;
int valvPin = 7;
int pumpPin = 8; 

int hallState = 0;
int hallPin = 10;

const long interval = 50;
unsigned long previousMillis = 0;

byte state = 0;
byte new_state = 0;
unsigned long currentMillis;

// sentinel for “no data / read error”
constexpr byte READ_FAIL = 0x80;

// returns 0b10000000 for failed reading
byte read_byte(){
  int i;
  if (!Serial.available() || (i = Serial.read()) < 0) return READ_FAIL; // failed reading
  return (byte) i;
}

void setup() 
{
  Serial.begin(9600);
  
  pinMode(chrgPin, OUTPUT);
  pinMode(valvPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(hallPin, INPUT);
  
  digitalWrite(chrgPin, 0);
  digitalWrite(valvPin, 0);
  digitalWrite(pumpPin, 0);

  digitalWrite(hallPin, 1);

}

void loop() 
{
  if((new_state = read_byte()) != READ_FAIL){
    byte changed_state = state^new_state;
    if(bitRead(changed_state, 0)) digitalWrite(chrgPin, bitRead(new_state, 0));
    if(bitRead(changed_state, 1)) digitalWrite(valvPin, bitRead(new_state, 1));
    if(bitRead(changed_state, 2)) digitalWrite(pumpPin, bitRead(new_state, 2));
    state = new_state;
  }
  
  hallState = digitalRead(hallPin);
  
  // send the values over serial periodically
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    Serial.print(hallState);      Serial.print(",");
    Serial.print("0b");
    Serial.println(state, BIN);
  }
}
